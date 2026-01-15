// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.kitbot;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.device.imu.IMU;
import frc.robot.drive.Drivetrain;

public class KitbotTankDrivetrain extends SubsystemBase implements Drivetrain {
	// private final SparkMax leftLeader;
	// private final SparkMax leftFollower;
	// private final SparkMax rightLeader;
	// private final SparkMax rightFollower;

	// Motor controller IDs for drivetrain motors
	public static final int LEFT_LEADER_ID = 1;
	public static final int LEFT_FOLLOWER_ID = 2;
	public static final int RIGHT_LEADER_ID = 3;
	public static final int RIGHT_FOLLOWER_ID = 4;

	public static final int LEFT_ENCODER_ID = 5;
	public static final int RIGHT_ENCODER_ID = 6;

	public static final int PIGEON_ID = 7;

	// Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
	// likelihood of tripping breakers or damaging CIM motors
	public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

	public static final double kGearRatio = 8.45;
	public static final Distance kWheelRadius = Inches.of(3);
	final int kCountsPerRev = 4096;  //Encoder counts per revolution of the motor shaft.
	final int k100msPerSecond = 10;

	public static final double WHEEL_CIRCUMFERENCE = kWheelRadius.in(Meters) * 2 * Math.PI;

	public static final double MAX_SPEED = 2; // m/s

	private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
	private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
	private final CANcoder leftEncoder = new CANcoder(LEFT_ENCODER_ID);
	
	private final DCMotor leftGearbox = DCMotor.getCIM(2);
	private final TalonSRXSimCollection leftLeaderSim = leftLeader.getSimCollection();
	private final TalonSRXSimCollection leftFollowerSim = leftFollower.getSimCollection();
	private final CANcoderSimState leftEncoderSim = leftEncoder.getSimState();

	private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
	private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);
	private final CANcoder rightEncoder = new CANcoder(RIGHT_ENCODER_ID);
	
	private final DCMotor rightGearbox = DCMotor.getCIM(2);
	private final TalonSRXSimCollection rightLeaderSim = rightLeader.getSimCollection();
	private final TalonSRXSimCollection rightFollowerSim = rightFollower.getSimCollection();
	private final CANcoderSimState rightEncoderSim = rightEncoder.getSimState();

	private final DutyCycleOut m_leftOut = new DutyCycleOut(0); // Initialize with 0% output
	private final DutyCycleOut m_rightOut = new DutyCycleOut(0); // Initialize with 0% output

	private final IMU imu = IMU.getKitbotIMU(PIGEON_ID);

	private final DifferentialDrive drive = new DifferentialDrive(leftLeader::set, rightLeader::set);
	// private final DifferentialDriveOdometry odometry;

	private final DifferentialDriveOdometry simOdometry;
	
	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(edu.wpi.first.math.util.Units.inchesToMeters(26.0));
	
	private final DifferentialDrivePoseEstimator poseEstimator;

	// Create the simulation model of our drivetrain.
	// https://andymark.com/products/am14u6-6-wheel-drop-center-robot-drive-base-2025-frc-kit-of-parts-drive-base
	private final DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
		KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
		KitbotGearing.k8p45,        // TODO this should be 8.46:1
		KitbotWheelSize.kSixInch,    // 6" diameter wheels.
		null                         // No measurement noise.
	);

	public KitbotTankDrivetrain() {
		this(new Pose2d());
	}

	// /**
	//  * Initialize Pigeon2 device from the configurator object
	//  * 
	//  * @param cfg Configurator of the Pigeon2 device
	//  */
	// private void initializePigeon2(Pigeon2Configurator cfg) {
	// 	var toApply = new Pigeon2Configuration();

	// 	/*
	// 		* User can change configs if they want, or leave this blank for factory-default
	// 		*/

	// 	cfg.apply(toApply);

	// 	/* And initialize yaw to 0 */
	// 	cfg.setYaw(0);
	// }

	// https://raw.githubusercontent.com/CrossTheRoadElec/Phoenix6-Examples/refs/heads/main/java/CommandBasedDrive/src/main/java/frc/robot/subsystems/DriveSubsystem.java
	// https://raw.githubusercontent.com/CrossTheRoadElec/Phoenix5-Examples/refs/heads/master/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java
	public KitbotTankDrivetrain(Pose2d startingPose) {
		// initializePigeon2(imu.getConfigurator());
		rightLeader.configFactoryDefault();
		rightFollower.configFactoryDefault();
		rightFollower.follow(rightLeader);
		rightFollower.setInverted(InvertType.FollowMaster);

		leftLeader.configFactoryDefault();
		leftFollower.configFactoryDefault();
		leftFollower.follow(leftLeader);
		leftFollower.setInverted(InvertType.FollowMaster);

		/* The left side is positive forward and sensor is in phase by default */
    leftLeader.setInverted(InvertType.None);
    leftLeader.setSensorPhase(false);
    /*
     * The right side sensor is also already in phase in phase but the
     * output needs to be inverted so positive is forward
     */
    rightLeader.setInverted(InvertType.InvertMotorOutput);
    rightLeader.setSensorPhase(false);

		/*
			* Set the update frequency of the main requests to 0 so updates are sent
			* immediately in the arcadeDrive method
			*/
		m_leftOut.UpdateFreqHz = 0;
		m_rightOut.UpdateFreqHz = 0;

		if(RobotBase.isSimulation()) {
			simOdometry = new DifferentialDriveOdometry(
				imu.getRotation2d(),
				driveSim.getLeftPositionMeters(),
				driveSim.getRightPositionMeters(),
				startingPose);
		} else {
			simOdometry = null;
		}
		
		poseEstimator = new DifferentialDrivePoseEstimator(
			kinematics,
			imu.getRotation2d(),
			rotationsToMeters(leftEncoder.getPosition().getValue()).in(Meters),
			rotationsToMeters(rightEncoder.getPosition().getValue()).in(Meters),
			startingPose);
	}

	@Override
	public void periodic() {
		poseEstimator.update(
			imu.getRotation2d(),
			rotationsToMeters(leftEncoder.getPosition().getValue()).in(Meters),
			rotationsToMeters(rightEncoder.getPosition().getValue()).in(Meters)
		);
	}

	@Override
	public void simulationPeriodic() {
		driveSim.setInputs(leftLeader.getMotorOutputVoltage(),
											 rightLeader.getMotorOutputVoltage());
		
		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		driveSim.update(0.02);

		// Update all of our sensors.
		leftLeaderSim.setQuadratureRawPosition(
			distanceToNativeUnits(driveSim.getLeftPositionMeters())
		);
		leftLeaderSim.setQuadratureVelocity(
			distanceToNativeUnits(driveSim.getLeftPositionMeters())
		);
		leftFollowerSim.setQuadratureRawPosition(
			distanceToNativeUnits(driveSim.getLeftPositionMeters())
		);
		leftFollowerSim.setQuadratureVelocity(
			distanceToNativeUnits(driveSim.getLeftPositionMeters())
		);
		leftEncoderSim.setRawPosition(
			metersToRotations(Meters.of(driveSim.getLeftPositionMeters()))
		);
		leftEncoderSim.setVelocity(
			metersToRotationsVel(MetersPerSecond.of(driveSim.getLeftVelocityMetersPerSecond()))
		);

		rightLeaderSim.setQuadratureRawPosition(
			distanceToNativeUnits(driveSim.getRightPositionMeters())
		);
		rightLeaderSim.setQuadratureVelocity(
			distanceToNativeUnits(driveSim.getRightPositionMeters())
		);
		rightFollowerSim.setQuadratureRawPosition(
			distanceToNativeUnits(driveSim.getRightPositionMeters())
		);
		rightFollowerSim.setQuadratureVelocity(
			distanceToNativeUnits(driveSim.getRightPositionMeters())
		);
		rightEncoderSim.setRawPosition(
			metersToRotations(Meters.of(driveSim.getRightPositionMeters()))
		);
		rightEncoderSim.setVelocity(
			metersToRotationsVel(MetersPerSecond.of(driveSim.getRightVelocityMetersPerSecond()))
		);

		imu.simSetRawYaw(driveSim.getHeading());

		SmartDashboard.putNumber("Sim left position", driveSim.getLeftPositionMeters());
		SmartDashboard.putNumber("Sim right position", driveSim.getRightPositionMeters());

		simOdometry.update(
			imu.getRotation2d(),
			driveSim.getLeftPositionMeters(),
			driveSim.getRightPositionMeters()
		);

		SmartDashboard.putNumber("Left voltage", leftLeader.get() * RobotController.getInputVoltage());
		SmartDashboard.putNumber("Right voltage", rightLeader.get() * RobotController.getInputVoltage());
		SmartDashboard.putNumber("Left encoder", rotationsToMeters(leftEncoder.getPosition().getValue()).in(Meters));
		SmartDashboard.putNumber("Right encoder", rotationsToMeters(rightEncoder.getPosition().getValue()).in(Meters));
	}

	// Command factory to create command to drive the robot with joystick inputs.
	public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
		return this.run(
				() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
	}

	/** See {@link DifferentialDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}

	/** See {@link DifferentialDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
	public void addVisionMeasurement(
			Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	public void resetFieldOrientation() {
		// TODO
	}

	public ChassisSpeeds getVelocities() {
		return null;
	}

	public void brake() {
		drive.stopMotor();
	}

	public void log() {
	}

	/** Get the estimated pose of the swerve drive on the field. */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/** The heading of the swerve drive's estimated pose on the field. */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	public void driveRobotCentric(ChassisSpeeds speeds) {
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		drive.tankDrive(wheelSpeeds.leftMetersPerSecond / MAX_SPEED, wheelSpeeds.rightMetersPerSecond / MAX_SPEED);
	}

	public Pose2d getSimPose() {
		return simOdometry.getPoseMeters();
	}

	private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * kWheelRadius.in(Meters));
    double motorRotations = wheelRotations * kGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * kWheelRadius.in(Meters));
    double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * kWheelRadius.in(Meters));
    return positionMeters;
  }

	private Distance rotationsToMeters(Angle rotations) {
		/* Apply gear ratio to input rotations */
		var gearedRadians = rotations.in(Radians) / kGearRatio;
		/* Then multiply the wheel radius by radians of rotation to get distance */
		return kWheelRadius.times(gearedRadians);
	}

	private Angle metersToRotations(Distance meters) {
		/* Divide the distance by the wheel radius to get radians */
		var wheelRadians = meters.in(Meters) / kWheelRadius.in(Meters);
		/* Then multiply by gear ratio to get rotor rotations */
		return Radians.of(wheelRadians * kGearRatio);
	}

	private LinearVelocity rotationsToMetersVel(AngularVelocity rotations) {
		/* Apply gear ratio to input rotations */
		var gearedRotations = rotations.in(RadiansPerSecond) / kGearRatio;
		/* Then multiply the wheel radius by radians of rotation to get distance */
		return kWheelRadius.per(Second).times(gearedRotations);
	}

	private AngularVelocity metersToRotationsVel(LinearVelocity meters) {
		/* Divide the distance by the wheel radius to get radians */
		var wheelRadians = meters.in(MetersPerSecond) / kWheelRadius.in(Meters);
		/* Then multiply by gear ratio to get rotor rotations */
		return RadiansPerSecond.of(wheelRadians * kGearRatio);
	}
}
