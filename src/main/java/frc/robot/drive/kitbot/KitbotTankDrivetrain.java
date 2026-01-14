// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.kitbot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

	public static final double WHEEL_CIRCUMFERENCE = edu.wpi.first.math.util.Units.inchesToMeters(6) * Math.PI;

	public static final double MAX_SPEED = 2; // m/s

	private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
	private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
	private final CANcoder leftEncoder = new CANcoder(LEFT_ENCODER_ID);
	
	private final DCMotor leftGearbox = DCMotor.getCIM(2);
	private final SparkMaxSim leftLeaderSim = new SparkMaxSim(leftLeader, leftGearbox);
	private final SparkMaxSim leftFollowerSim = new SparkMaxSim(leftFollower, leftGearbox);
	private final CANcoderSimState leftEncoderSim = leftEncoder.getSimState();

	private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
	private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);
	private final CANcoder rightEncoder = new CANcoder(RIGHT_ENCODER_ID);
	
	private final DCMotor rightGearbox = DCMotor.getCIM(2);
	private final SparkMaxSim rightLeaderSim = new SparkMaxSim(rightLeader, rightGearbox);
	private final SparkMaxSim rightFollowerSim = new SparkMaxSim(rightFollower, rightGearbox);
	private final CANcoderSimState rightEncoderSim = rightEncoder.getSimState();

	private final Pigeon2 imu = new Pigeon2(PIGEON_ID);
	private final Pigeon2SimState imuSim = imu.getSimState();

	private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);;
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

	public KitbotTankDrivetrain(Pose2d startingPose) {
		// Set can timeout. Because this project only sets parameters once on
		// construction, the timeout can be long without blocking robot operation. Code
		// which sets or gets parameters during operation may need a shorter timeout.
		leftLeader.setCANTimeout(250);
		rightLeader.setCANTimeout(250);
		leftFollower.setCANTimeout(250);
		rightFollower.setCANTimeout(250);

		// Create the configuration to apply to motors. Voltage compensation
		// helps the robot perform more similarly on different
		// battery voltages (at the cost of a little bit of top speed on a fully charged
		// battery). The current limit helps prevent tripping
		// breakers.
		SparkMaxConfig config = new SparkMaxConfig();
		config.voltageCompensation(12);
		config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

		// Set configuration to follow each leader and then apply it to corresponding
		// follower. Resetting in case a new controller is swapped
		// in and persisting in case of a controller reset due to breaker trip
		config.follow(leftLeader);
		leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		config.follow(rightLeader);
		rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Remove following, then apply config to right leader
		config.disableFollowerMode();
		rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		// Set config to inverted and then apply to left leader. Set Left side inverted
		// so that postive values drive both sides forward
		config.inverted(true);
		leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
		// odometry = new DifferentialDriveOdometry(
		// 	imu.getRotation2d(),
		// 	leftEncoder.getPosition().getValue().in(Units.Rotation) * WHEEL_CIRCUMFERENCE,
		// 	rightEncoder.getPosition().getValue().in(Units.Rotation) * WHEEL_CIRCUMFERENCE,
		// 	startingPose);

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
			leftEncoder.getPosition().getValue().in(Units.Rotation) * WHEEL_CIRCUMFERENCE,
			rightEncoder.getPosition().getValue().in(Units.Rotation) * WHEEL_CIRCUMFERENCE,
			startingPose);
	}

	@Override
	public void periodic() {
		poseEstimator.update(
			imu.getRotation2d(),
			leftEncoder.getPosition().getValue().in(Units.Rotation) * WHEEL_CIRCUMFERENCE,
			rightEncoder.getPosition().getValue().in(Units.Rotation) * WHEEL_CIRCUMFERENCE
		);
	}

	@Override
	public void simulationPeriodic() {
		// Set the inputs to the system. Note that we need to convert
		// the [-1, 1] PWM signal to voltage by multiplying it by the
		// robot controller voltage.
		driveSim.setInputs(leftLeader.get() * RobotController.getInputVoltage(),
											 rightLeader.get() * RobotController.getInputVoltage());
		
		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		driveSim.update(0.02);

		// Update all of our sensors.
		leftEncoderSim.setRawPosition(driveSim.getLeftPositionMeters() / WHEEL_CIRCUMFERENCE);
		leftEncoderSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond() / WHEEL_CIRCUMFERENCE);
		leftLeaderSim.setMotorCurrent(driveSim.getLeftCurrentDrawAmps());
		leftFollowerSim.setMotorCurrent(driveSim.getLeftCurrentDrawAmps());

		rightEncoderSim.setRawPosition(driveSim.getRightPositionMeters() / WHEEL_CIRCUMFERENCE);
		rightEncoderSim.setVelocity(driveSim.getRightVelocityMetersPerSecond() / WHEEL_CIRCUMFERENCE);
		rightLeaderSim.setMotorCurrent(driveSim.getRightCurrentDrawAmps());
		rightFollowerSim.setMotorCurrent(driveSim.getRightCurrentDrawAmps());

		imuSim.setRawYaw(-driveSim.getHeading().getDegrees());

		SmartDashboard.putNumber("Sim left position", driveSim.getLeftPositionMeters());
		SmartDashboard.putNumber("Sim right position", driveSim.getRightPositionMeters());

		simOdometry.update(
			imu.getRotation2d(),
			driveSim.getLeftPositionMeters(),
			driveSim.getRightPositionMeters()
		);

		SmartDashboard.putNumber("Left voltage", leftLeader.get() * RobotController.getInputVoltage());
		SmartDashboard.putNumber("Right voltage", rightLeader.get() * RobotController.getInputVoltage());
		SmartDashboard.putNumber("Left encoder", leftLeader.getEncoder().getPosition());
		SmartDashboard.putNumber("Right encoder", rightLeader.getEncoder().getPosition());
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
}
