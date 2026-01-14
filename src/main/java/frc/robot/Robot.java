// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.DriveUtility;
import frc.robot.drive.kitbot.KitbotTankDrivetrain;
import frc.robot.drive.pvswerve.SimSwerveConstants;
import frc.robot.drive.pvswerve.SimSwerveDrivetrain;
import frc.robot.localization.Vision;
import frc.robot.subsystems.GamepieceLauncher;
import frc.robot.subsystems.KitbotFuelSubsystem;
import frc.robot.Constants.*;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	public static final double kMaxLinearSpeed = 2; // m/s
	public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);

	// The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

	// public final Drivetrain drivetrain = DriveUtility.makeDrivetrain(this::resetPose);
	public final KitbotTankDrivetrain drivetrain = new KitbotTankDrivetrain();
	public final KitbotFuelSubsystem fuelSubsystem = new KitbotFuelSubsystem();

	public final Vision vision = new Vision(drivetrain::addVisionMeasurement);

	private Field2d debugField = new Field2d();

	public final double DEADBAND = 0.1;

	// public SendableChooser<Command> autoChooser;

	public Robot() {
		// autoChooser = AutoBuilder.buildAutoChooser();
		// SmartDashboard.putData("Auto Chooser", autoChooser);

		if(RobotBase.isReal()) {
			SmartDashboard.putData("VisionSystemSim-main/Sim Field", debugField);
		}

		configureBindings();
	}

	private void configureBindings() {
		// While the left bumper on operator controller is held, intake Fuel
		operatorController.leftBumper()
				.whileTrue(fuelSubsystem.runEnd(() -> fuelSubsystem.intake(), () -> fuelSubsystem.stop()));
		// While the right bumper on the operator controller is held, spin up for 1
		// second, then launch fuel. When the button is released, stop.
		operatorController.rightBumper()
				.whileTrue(fuelSubsystem.spinUpCommand().withTimeout(KitbotFuelSubsystem.SPIN_UP_SECONDS)
						.andThen(fuelSubsystem.launchCommand())
						.finallyDo(() -> fuelSubsystem.stop()));
		// While the A button is held on the operator controller, eject fuel back out
		// the intake
		operatorController.a()
				.whileTrue(fuelSubsystem.runEnd(() -> fuelSubsystem.eject(), () -> fuelSubsystem.stop()));

		// Set the default command for the drive subsystem to the command provided by
		// factory with the values provided by the joystick axes on the driver
		// controller. The Y axis of the controller is inverted so that pushing the
		// stick away from you (a negative value) drives the robot forwards (a positive
		// value). The X-axis is also inverted so a positive value (stick to the right)
		// results in clockwise rotation (front of the robot turning right). Both axes
		// are also scaled down so the rotation is more easily controllable.
		// drivetrain.setDefaultCommand(
		// 		drivetrain.driveArcade(
		// 				() -> -driverController.getLeftY() * OperatorConstants.DRIVE_SCALING,
		// 				() -> -driverController.getRightX() * OperatorConstants.ROTATION_SCALING));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		// if(RobotBase.isSimulation()) {
		// 	((SimSwerveDrivetrain)drivetrain).periodic();
		// }

		// Update vision
		vision.periodic();

		// Test/Example only!
		// Apply an offset to pose estimator to test vision correction
		// You probably don't want this on a real robot, just delete it.
		// if (controller.getBButtonPressed()) {
		// 	var disturbance =
		// 		new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
		// 	drivetrain.resetPose(drivetrain.getPose().plus(disturbance), false);
		// }

		// debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());

		// Log values to the dashboard
		drivetrain.log();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		drivetrain.brake();
	}

	@Override
	public void autonomousInit() {
		// m_autonomousCommand = autoChooser.getSelected();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		// resetPose();
	}

	public double deadband(double input) {
		return Math.min(DEADBAND, input);
	}

	@Override
	public void teleopPeriodic() {
		// Calculate drivetrain commands from Joystick values
		double forward = -driverController.getLeftY() * kMaxLinearSpeed;
		double strafe = -driverController.getLeftX() * kMaxLinearSpeed;
		double turn = -driverController.getRightX() * kMaxAngularSpeed;

		// Command drivetrain motors based on target speeds
		drivetrain.driveRobotCentric(forward, strafe, turn);
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {
		// SimSwerveDrivetrain simDrivetrain = (SimSwerveDrivetrain)drivetrain;
		KitbotTankDrivetrain simDrivetrain = (KitbotTankDrivetrain)drivetrain;
		// simDrivetrain.simulationPeriodic();
		// // Update camera simulation
		vision.simulationPeriodic(simDrivetrain.getSimPose());

		debugField = vision.getSimDebugField();
		// debugField.getObject("Robot").setPose(simDrivetrain.getSimPose());
		debugField.getObject("EstimatedRobot").setPose(simDrivetrain.getPose());
		// debugField.getObject("EstimatedRobotModules").setPoses(simDrivetrain.getModulePoses());

		// // Update gamepiece launcher simulation
		// gpLauncher.simulationPeriodic();

		// // Calculate battery voltage sag due to current draw
		// double batteryVoltage =
		// 	BatterySim.calculateDefaultBatteryLoadedVoltage(simDrivetrain.getCurrentDraw());

		// // Using max(0.1, voltage) here isn't a *physically correct* solution,
		// // but it avoids problems with battery voltage measuring 0.
		// RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
	}

	public void resetPose() {
		resetPose(new Pose2d(1, 1, new Rotation2d()));
	}

	public void resetPose(Pose2d startPose) {
		// if(RobotBase.isSimulation()) {
		// 	((SimSwerveDrivetrain)drivetrain).resetPose(startPose, true);
		// }
		vision.resetSimPose(startPose);
	}
}
