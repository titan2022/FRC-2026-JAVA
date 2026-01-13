// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.kitbot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.drive.kitbot.KitbotDriveConstants.*;

public class KitbotTankDrivetrain extends SubsystemBase {
	// private final SparkMax leftLeader;
	// private final SparkMax leftFollower;
	// private final SparkMax rightLeader;
	// private final SparkMax rightFollower;

	private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
	private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
	
	private final DCMotor leftGearbox = DCMotor.getCIM(2);
	private final SparkMaxSim leftLeaderSim = new SparkMaxSim(leftLeader, leftGearbox);
	private final SparkMaxSim leftFollowerSim = new SparkMaxSim(leftFollower, leftGearbox);
	
	private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
	private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);
	private final DCMotor rightGearbox = DCMotor.getCIM(2);
	private final SparkMaxSim rightLeaderSim = new SparkMaxSim(rightLeader, rightGearbox);
	private final SparkMaxSim rightFollowerSim = new SparkMaxSim(rightFollower, rightGearbox);

	private final Pigeon2 imu = new Pigeon2(PIGEON_ID);
	private final Pigeon2SimState imuSim = imu.getSimState();

	private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);;
	private DifferentialDriveOdometry odometry;

	private final Field2d field = new Field2d();

	private Pose2d pose = null;

	// Create the simulation model of our drivetrain.
	// https://andymark.com/products/am14u6-6-wheel-drop-center-robot-drive-base-2025-frc-kit-of-parts-drive-base
	private final DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
		KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
		KitbotGearing.k8p45,        // TODO this should be 8.46:1
		KitbotWheelSize.kSixInch,    // 6" diameter wheels.
		null                         // No measurement noise.
	);


	public KitbotTankDrivetrain() {
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
	
		odometry = null;
		if(RobotBase.isSimulation()) {
			odometry = new DifferentialDriveOdometry(
				imu.getRotation2d(),
				driveSim.getLeftPositionMeters(),
				driveSim.getRightPositionMeters());
		}

		SmartDashboard.putData("Field", field);
	}

	@Override
	public void periodic() {
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
		leftLeaderSim.setPosition(driveSim.getLeftPositionMeters());
		leftLeaderSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond());
		leftLeaderSim.setMotorCurrent(driveSim.getLeftCurrentDrawAmps());

		rightLeaderSim.setPosition(driveSim.getRightPositionMeters());
		rightLeaderSim.setVelocity(driveSim.getRightVelocityMetersPerSecond());
		rightLeaderSim.setMotorCurrent(driveSim.getRightCurrentDrawAmps());

		imuSim.setRawYaw(-driveSim.getHeading().getDegrees());

		SmartDashboard.putNumber("Sim left position", driveSim.getLeftPositionMeters());
		SmartDashboard.putNumber("Sim right position", driveSim.getRightPositionMeters());

		pose = odometry.update(
			imu.getRotation2d(),
			driveSim.getLeftPositionMeters(),
			driveSim.getRightPositionMeters()
		);

		field.setRobotPose(odometry.getPoseMeters());

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

	public void brake() {
		drive.stopMotor();
	}
}
