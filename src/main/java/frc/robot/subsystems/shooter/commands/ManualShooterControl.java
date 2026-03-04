package frc.robot.subsystems.shooter.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.shooter.ShooterYaw;

public class ManualShooterControl extends Command {
  private final ShooterFlywheel shooterFlywheel;
  private final ShooterPitch shooterPitch;
  private final ShooterYaw shooterYaw;

  private double flywheelSpeed = 0.0;
  private static final double FLYWHEEL_SPEED_INCREMENT = 0.1;
  private static final double RIGHT_FLYWHEEL_SPEED = 3.0;
  private static final double LEFT_FLYWHEEL_SPEED = 3.0;

  private final CommandXboxController operatorController;
  private static final double DEADBAND = 0.7;

  public ManualShooterControl(ShooterFlywheel shooterFlywheel, ShooterPitch shooterPitch, ShooterYaw shooterYaw, CommandXboxController operatorController) {
    this.shooterFlywheel = shooterFlywheel;
    this.shooterPitch = shooterPitch;
    this.shooterYaw = shooterYaw;

    this.operatorController = operatorController;

    addRequirements(shooterFlywheel, shooterPitch, shooterYaw);
  }

  private static double applyDeadband(double joy, double deadband) {
    return Math.abs(joy) < deadband ? 0 : joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    operatorController.pov(0).onTrue(new InstantCommand(() -> {
      flywheelSpeed += FLYWHEEL_SPEED_INCREMENT;
      shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }));
    operatorController.pov(180).onTrue(new InstantCommand(() -> {
      flywheelSpeed += FLYWHEEL_SPEED_INCREMENT;
      shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }));

    operatorController.pov(90).onTrue(new InstantCommand(() -> {
      flywheelSpeed = LEFT_FLYWHEEL_SPEED;
      shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }));
    operatorController.pov(270).onTrue(new InstantCommand(() -> {
      flywheelSpeed = RIGHT_FLYWHEEL_SPEED;
      shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawMagnitude = Math.hypot(operatorController.getLeftX(), operatorController.getLeftY());
    double yawAngle = (Math.toDegrees(Math.atan2(operatorController.getLeftY(), operatorController.getLeftX())) % 360 + 360) % 360;

    if (yawMagnitude >= 0.7) {
      shooterYaw.setAngularPosition(yawAngle);
    }

    double pitchMagnitude = Math.hypot(operatorController.getRightX(), operatorController.getRightY());
    double pitchAngle = (Math.toDegrees(Math.atan2(operatorController.getRightY(), operatorController.getRightX())) % 360 + 360) % 360;

    if (pitchMagnitude >= 0.7) {
      shooterPitch.setAngularPosition(pitchAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterYaw.stop();
    shooterPitch.stop();
    shooterFlywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
