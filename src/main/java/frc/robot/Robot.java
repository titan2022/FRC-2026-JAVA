// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.drive.ctre.CTRESwerveTelemetry;
import frc.robot.drive.ctre.commands.DrivingCommand;
import frc.robot.drive.ctre.commands.DriveToPose;
import frc.robot.localization.Vision;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.indexer.Spindexer;
import frc.robot.subsystems.indexer.VerticalIndexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Pinion;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.shooter.ShooterYaw;
import frc.robot.subsystems.shooter.commands.ManualShooterControl;

public class Robot extends TimedRobot {	
	// private final CTRESwerveTelemetry logger = new CTRESwerveTelemetry(DriverConstants.MAX_SPEED);

	private Command m_autonomousCommand;

	public final CommandXboxController driveController = new CommandXboxController(0);
	public final CommandXboxController operatorController = new CommandXboxController(1);

	public final CTRESwerveDrivetrain drivetrain = new CTRESwerveDrivetrain();

	// public final Vision vision = new Vision(drivetrain::addVisionMeasurement);

	public final ShooterPitch shooterPitch = new ShooterPitch();
	// public final ShooterYaw shooterYaw = new ShooterYaw();
	public final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();

	// public final Spindexer spindexer = new Spindexer();
	// public final VerticalIndexer verticalIndexer = new VerticalIndexer();

	public final Intake intake = new Intake(drivetrain);
	public final Pinion pinion = new Pinion();

	// public final Climb climb = new Climb();

	private final DrivingCommand drivingCommand = new DrivingCommand(drivetrain, driveController);
	// private final ManualShooterControl manualShooterControl = new ManualShooterControl(
	// 	shooterFlywheel, shooterPitch, // shooterYaw, 
	// 	operatorController,
	// 	drivetrain,
	// 	intake //,
	// 	// 1000 // every 1s
	// );

	private static final double kCancelStickThreshold = 0.18;

	private static final PathConstraints kPathfindConstraints = new PathConstraints(
		3.0,
		3.0,
		Math.toRadians(540),
		Math.toRadians(720)
	);

	public SendableChooser<Command> autoChooser;

	public Robot() {
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		DogLog.setOptions(new DogLogOptions()
						.withLogExtras(false)
						.withCaptureDs(true)
						.withNtPublish(true)
						.withCaptureNt(true));
		// DogLog.setPdh(new PowerDistribution());

		// drivetrain.registerTelemetry(logger::telemeterize);

		// Right now we don't want it to be default command as stop, since
		// the normal commands do not continue indefinitely.

		// shooterPitch.setDefaultCommand(shooterPitch.stopCommand());
		// shooterFlywheel.setDefaultCommand(shooterFlywheel.stopCommand());
		// spindexer.setDefaultCommand(spindexer.stopCommand());
		// verticalIndexer.setDefaultCommand(verticalIndexer.stopCommand());

		// intake.setDefaultCommand(intake.stopCommand());
		// pinion.setDefaultCommand(pinion.stopCommand());

		resetPose();

		configureBindings();
	}

	public void configureBindings() {
		// // If you modify these controls please update the diagram at.:
    // //   current state: https://docs.google.com/drawings/d/1_Lk5ZLvhy3-GtpytwQDFhX6L3Q5EoNN4N0K72jPGByc/edit
    // //            plan: https://docs.google.com/drawings/d/18_HOTw2HHTe6EamlZadLaGDxj3c08HJma-SCfwRxWIQ/edit

		// drivetrain.setDefaultCommand(drivingCommand);

		// driveController.x().onTrue(
		// 	new DriveToPose(
		// 		drivetrain,
		// 		new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(180.0)),
		// 		kPathfindConstraints
		// 	)
		// );	
		// new Trigger(() ->
		// 	Math.abs(driveController.getLeftX()) > kCancelStickThreshold ||
		// 	Math.abs(driveController.getLeftY()) > kCancelStickThreshold ||
		// 	Math.abs(driveController.getRightX()) > kCancelStickThreshold
		// ).onTrue(Commands.runOnce(drivetrain::cancelActiveDrive));
		
		operatorController.a().onTrue(pinion.retractIntakeCommand().alongWith(intake.intakeCommand()));
		operatorController.b().onTrue(pinion.extendIntakeCommand().alongWith(intake.stopCommand()));

		operatorController.leftTrigger().onTrue(shooterPitch.setAngularPositionCommand(ShootingConstants.shootToAllianceArea_firingPitch)
			.alongWith(shooterFlywheel.setAngularPositionCommand(ShootingConstants.shootToAllianceArea_shooterYaw)));

		// operatorController.x().onTrue(climb.climbDownCommand());
		// operatorController.y().onTrue(climb.climbUpCommand());

		// operatorController.a().onTrue(shooterYaw.setAngularPositionCommand(0.5));
		// operatorController.b().onTrue(shooterYaw.setAngularPositionCommand(1.0));

		// operatorController.leftBumper().onTrue(manualShooterControl);
		// operatorController.rightBumper().onTrue(
		// 	shooterFlywheel.stopCommand()
		// 		.alongWith(shooterPitch.stopCommand())
		// 		// .alongWith(shooterYaw.stopCommand())
		// );

		// Following are used for testing individual subsystems.

		// operatorController.a().whileTrue(climb.setLinearPositionCommand(0));
		// operatorController.b().whileTrue(climb.setLinearPositionCommand(0.3));
		// operatorController.x().whileTrue(climb.setLinearPositionCommand(0.7));
		// operatorController.y().whileTrue(climb.setLinearPositionCommand(1));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		// if(RobotBase.isSimulation()) {
		// 	((SimSwerveDrivetrain)drivetrain).periodic();
		// }

		// Update vision
		// vision.periodic();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		drivetrain.brake();
		drivetrain.cancelActiveDrive();

	}

	@Override
	public void autonomousInit() {
		drivingCommand.resetAlliance();

		m_autonomousCommand = autoChooser.getSelected();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		drivingCommand.resetAlliance();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		drivetrain.cancelActiveDrive();

		resetPose();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	private final StructArrayPublisher<Pose3d> fuelPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Robot/Field/Fuel", Pose3d.struct)
      .publish();

	@Override
	public void simulationInit() {
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	@Override
	public void simulationPeriodic() {
		// Update camera simulation
		// vision.simulationPeriodic(drivetrain.getSimPose());

		// Get the positions of the fuel (both on the field and in the air)
		fuelPoses.accept(SimulatedArena.getInstance()
					.getGamePiecesByType("Fuel")
					.stream()
					.map(x -> x.getPose3d())
					.toArray(size -> new Pose3d[size]));

		SimulatedArena.getInstance().simulationPeriodic();	
	}

	public void resetPose() {
		resetPose(new Pose2d(1, 1, new Rotation2d()));
	}

	public void resetPose(Pose2d startPose) {
		drivetrain.resetPose(startPose);
		// vision.resetSimPose(startPose);
	}
}
