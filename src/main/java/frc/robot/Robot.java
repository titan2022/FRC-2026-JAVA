// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;

import static frc.robot.ToSI.*;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.DriveUtility;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.drive.ctre.CTRESwerveTelemetry;
import frc.robot.drive.ctre.TunerConstants;
import frc.robot.drive.sim.SimSwerveConstants;
import frc.robot.drive.sim.SimSwerveDrivetrain;
import frc.robot.localization.Vision;
import frc.robot.subsystems.intake.IntakePinion;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.shooter.ShooterYaw;

public class Robot extends TimedRobot {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate =
					RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
	private final CTRESwerveTelemetry logger = new CTRESwerveTelemetry(MaxSpeed);

	private Command m_autonomousCommand;

	public final CommandXboxController controller = new CommandXboxController(0);

	public final CTRESwerveDrivetrain drivetrain = new CTRESwerveDrivetrain();

	public final Vision vision = new Vision(drivetrain::addVisionMeasurement);

	public final ShooterPitch shooterPitch = new ShooterPitch();
	public final ShooterYaw shooterYaw = new ShooterYaw();
	public final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();

	public final IntakePinion intakePinion = new IntakePinion();

	public SendableChooser<Command> autoChooser;

	public Robot() {
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		DogLog.setOptions(new DogLogOptions()
						.withLogExtras(true)
						.withCaptureDs(true)
						.withNtPublish(true)
						.withCaptureNt(true));
		DogLog.setPdh(new PowerDistribution());

		drivetrain.registerTelemetry(logger::telemeterize);

		resetPose();

		configureBindings();
	}

	public void configureBindings() {
		controller.a().whileTrue(intakePinion.setLinearPositionCommand(0*m));
		controller.b().whileTrue(intakePinion.setLinearPositionCommand(0.3*m));
		controller.x().whileTrue(intakePinion.setLinearPositionCommand(0.7*m));
		controller.y().whileTrue(intakePinion.setLinearPositionCommand(1*m));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		// if(RobotBase.isSimulation()) {
		// 	((SimSwerveDrivetrain)drivetrain).periodic();
		// }

		// Update vision
		vision.periodic();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		drivetrain.brake();
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
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		resetPose();
	}

	@Override
	public void teleopPeriodic() {
		// Calculate drivetrain commands from Joystick values
		double forward = -controller.getLeftY() * SimSwerveConstants.Swerve.kMaxLinearSpeed;
		double strafe = -controller.getLeftX() * SimSwerveConstants.Swerve.kMaxLinearSpeed;
		double turn = -controller.getRightX() * SimSwerveConstants.Swerve.kMaxAngularSpeed;

		// Command drivetrain motors based on target speeds
		drivetrain.driveRobotCentric(forward, strafe, turn);

		// Calculate whether the gamepiece launcher runs based on our global pose estimate.
		var curPose = drivetrain.getPose();
		var shouldRun = (curPose.getY() > 2.0 && curPose.getX() < 4.0); // Close enough to blue speaker
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
		// Update camera simulation
		vision.simulationPeriodic(drivetrain.getSimPose());

		SimulatedArena.getInstance().simulationPeriodic();	
	}

	public void resetPose() {
		resetPose(new Pose2d(1, 1, new Rotation2d()));
	}

	public void resetPose(Pose2d startPose) {
		drivetrain.resetPose(startPose);
		vision.resetSimPose(startPose);
	}
}
