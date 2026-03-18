package frc.robot.commands;

import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.HubStatus;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.shooter.StaticShotCalculator;

public final class ShooterCommands {
  private ShooterCommands() {}


  // don't remember if there was an other way we were supposed to have pid constants
  private static final double HEADING_kP = 5.0;
  private static final double HEADING_kI = 0.0;
  private static final double HEADING_kD = 0.20;

  private static final double MAX_AIM_OMEGA_RAD_PER_SEC = 4.0;

  private static final double HEADING_TOLERANCE_DEG = 2.0;
  private static final double PITCH_TOLERANCE_DEG = 1.0;
  private static final double FLYWHEEL_TOLERANCE_RPS = 3.0;

  public static Command staticAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterPitch pitch,
      ShooterFlywheel flywheel) {

    return staticShoot(
        drivetrain,
        pitch,
        flywheel,
        () -> false,
        () -> {},
        () -> {});
  }

  public static Command staticShoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterPitch pitch,
      ShooterFlywheel flywheel,
      BooleanSupplier fireRequest,
      Runnable feed,
      Runnable stopFeed) {

    StaticShotCalculator calculator = new StaticShotCalculator();

    PIDController headingController =
        new PIDController(HEADING_kP, HEADING_kI, HEADING_kD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Math.toRadians(HEADING_TOLERANCE_DEG));

    return new FunctionalCommand(
        () -> {
          headingController.reset();
          stopFeed.run();
        },
        () -> {
          var shot = calculator.calculate(drivetrain.getPose());

          if (!shot.valid()) {
            drivetrain.brake();
            flywheel.stopShooter();
            stopFeed.run();
            DogLog.log("Shooter/StaticShot/Ready", false);
            DogLog.log("Shooter/StaticShot/HubActive", HubStatus.isOwnHubActive());
            return;
          }

          pitch.setTargetDegrees(shot.pitchDegrees());
          flywheel.setTargetRps(shot.flywheelRps());

          double headingErrorRad =
              MathUtil.angleModulus(
                  shot.driveHeading().minus(drivetrain.getHeading()).getRadians());

          double omegaCmd =
              MathUtil.clamp(
                  headingController.calculate(
                      drivetrain.getHeading().getRadians(),
                      shot.driveHeading().getRadians()),
                  -MAX_AIM_OMEGA_RAD_PER_SEC,
                  MAX_AIM_OMEGA_RAD_PER_SEC);

          // Static shot  rotate robot to target
          drivetrain.driveRobotCentric(0.0, 0.0, omegaCmd);

          boolean headingReady = Math.abs(headingErrorRad) <= Math.toRadians(HEADING_TOLERANCE_DEG);
          boolean pitchReady = pitch.atTargetDegrees(PITCH_TOLERANCE_DEG);
          boolean flywheelReady = flywheel.atTargetRps(FLYWHEEL_TOLERANCE_RPS);
          boolean hubActive = HubStatus.isOwnHubActive();

          boolean readyToShoot = headingReady && pitchReady && flywheelReady && hubActive;

          DogLog.log("Shooter/StaticShot/HeadingErrorDeg", Math.toDegrees(headingErrorRad), "deg");
          DogLog.log("Shooter/StaticShot/HeadingReady", headingReady);
          DogLog.log("Shooter/StaticShot/PitchReady", pitchReady);
          DogLog.log("Shooter/StaticShot/FlywheelReady", flywheelReady);
          DogLog.log("Shooter/StaticShot/HubActive", hubActive);
          DogLog.log("Shooter/StaticShot/Ready", readyToShoot);

          if (fireRequest.getAsBoolean() && readyToShoot) {
            feed.run();
          } else {
            stopFeed.run();
          }
        },
        interrupted -> {
          stopFeed.run();
          drivetrain.brake();
          flywheel.stopShooter();
        },
        () -> false,
        pitch,
        flywheel,
        drivetrain)
    .withName("StaticShoot");
  }
}