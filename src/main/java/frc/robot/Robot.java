// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.DriveUtility;
import frc.robot.drive.sim.SimSwerveConstants;
import frc.robot.drive.sim.SimSwerveDrivetrain;
import frc.robot.localization.Vision;
import frc.robot.subsystems.GamepieceLauncher;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.ctre.TunerConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Robot extends TimedRobot {
	private static final double kDeadband = 0.08;

	private static final double kCancelStickThreshold = 0.18;

	private static final PathConstraints kPathfindConstraints = new PathConstraints(
    3.0, 3.0,
    Units.degreesToRadians(540),
    Units.degreesToRadians(720)
	);

	private Command m_driveToPoseCmd = null;
	private Command m_autonomousCommand;

	public final CommandXboxController controller = new CommandXboxController(0);

	public final CommandSwerveDrivetrain drivetrain = DriveUtility.makeDrivetrain(this::resetPose);

	public final Vision vision = new Vision(drivetrain::addVisionMeasurement);

	public final GamepieceLauncher gpLauncher = new GamepieceLauncher();

	public SendableChooser<Command> autoChooser;

	public Robot() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PathfindingCommand.warmupCommand().schedule();

    CommandScheduler.getInstance().setDefaultCommand(
        drivetrain,
        Commands.run(() -> {
            double maxLin = RobotBase.isReal()
                ? TunerConstants.kMaxLinearSpeedMps
                : SimSwerveConstants.Swerve.kMaxLinearSpeed;

            double maxAng = SimSwerveConstants.Swerve.kMaxAngularSpeed;

            double forward = -MathUtil.applyDeadband(controller.getLeftY(), kDeadband) * maxLin;
            double strafe  = -MathUtil.applyDeadband(controller.getLeftX(), kDeadband) * maxLin;
            double turn    = -MathUtil.applyDeadband(controller.getRightX(), kDeadband) * maxAng;

            drivetrain.driveRobotCentric(forward, strafe, turn);
        }, drivetrain)
    );

	configureBindings();

}

 public void configureBindings() {
        Pose2d targetBluePose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(180.0));

        controller.a().onTrue(Commands.runOnce(() -> startDriveToPose(targetBluePose)));
        controller.b().onTrue(Commands.runOnce(this::cancelDriveToPose));

        new Trigger(this::stickMovedForCancel)
            .onTrue(Commands.runOnce(this::cancelDriveToPose));

 }
private void startDriveToPose(Pose2d targetBluePose) {
        cancelDriveToPose();

        m_driveToPoseCmd = AutoBuilder
            .pathfindToPoseFlipped(targetBluePose, kPathfindConstraints, 0.0)
            .andThen(Commands.runOnce(drivetrain::brake, drivetrain));

        m_driveToPoseCmd.schedule();
    }

    private void cancelDriveToPose() {
		if (m_driveToPoseCmd != null) {
        	m_driveToPoseCmd.cancel();
        	m_driveToPoseCmd = null;
    	}
    }

    private boolean stickMovedForCancel() {
        return Math.abs(controller.getLeftX()) > kCancelStickThreshold
            || Math.abs(controller.getLeftY()) > kCancelStickThreshold
            || Math.abs(controller.getRightX()) > kCancelStickThreshold;
    }



	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		if(RobotBase.isSimulation()) {
			((SimSwerveDrivetrain)drivetrain).periodic();
		}

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

		// Log values to the dashboard
		drivetrain.log();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		drivetrain.brake();
		cancelDriveToPose();

    }
	

	@Override
	public void autonomousInit() {
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
    cancelDriveToPose();


    AutoBuilder.resetOdom(new Pose2d(1, 1, new Rotation2d())).schedule();

    if (m_driveToPoseCmd != null) {
        m_driveToPoseCmd.cancel();
        m_driveToPoseCmd = null;
    }

    if (RobotBase.isSimulation()) {
        resetPose();
    }
}


	@Override
public void teleopPeriodic() {
    Pose2d curPose = drivetrain.getPose();
        boolean shouldRun = (curPose.getY() > 2.0 && curPose.getX() < 4.0);
        gpLauncher.setRunning(shouldRun);
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
		SimSwerveDrivetrain simDrivetrain = (SimSwerveDrivetrain)drivetrain;
		simDrivetrain.simulationPeriodic();
		// Update camera simulation
		vision.simulationPeriodic(simDrivetrain.getSimPose());

		var debugField = vision.getSimDebugField();
		debugField.getObject("EstimatedRobot").setPose(simDrivetrain.getPose());
		debugField.getObject("EstimatedRobotModules").setPoses(simDrivetrain.getModulePoses());

		// Update gamepiece launcher simulation
		gpLauncher.simulationPeriodic();

		// Calculate battery voltage sag due to current draw
		var batteryVoltage =
			BatterySim.calculateDefaultBatteryLoadedVoltage(simDrivetrain.getCurrentDraw());

		// Using max(0.1, voltage) here isn't a *physically correct* solution,
		// but it avoids problems with battery voltage measuring 0.
		RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
	}

	public void resetPose() {
		resetPose(new Pose2d(1, 1, new Rotation2d()));
	}

	public void resetPose(Pose2d startPose) {
		if(RobotBase.isSimulation()) {
			((SimSwerveDrivetrain)drivetrain).resetPose(startPose, true);
		}
		vision.resetSimPose(startPose);
	}

	
}
