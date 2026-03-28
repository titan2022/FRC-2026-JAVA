package frc.robot.subsystems.shooter.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.shooter.ShooterYaw;

public class ManualShooterControl extends Command {
  private final ShooterFlywheel shooterFlywheel;
  private final ShooterPitch shooterPitch;
  // private final ShooterYaw shooterYaw;

  // private final CTRESwerveDrivetrain drivetrain;
  // private final Intake intake;

  // private long counter;
  // private final long maxCounterValue;

  private double flywheelSpeed = 0.0;
  private static final double FLYWHEEL_SPEED_INCREMENT = 0.1;
  private static final double RIGHT_FLYWHEEL_SPEED = 3.0;
  private static final double LEFT_FLYWHEEL_SPEED = 3.0;

  private final CommandXboxController operatorController;
  private static final double DEADBAND = 0.7;

  public ManualShooterControl(
    ShooterFlywheel shooterFlywheel, 
    ShooterPitch shooterPitch, 
    // ShooterYaw shooterYaw, 
    CommandXboxController operatorController, 
    // CTRESwerveDrivetrain drivetrain, 
    Intake intake //, 
    // long period
  ) {
    this.shooterFlywheel = shooterFlywheel;
    this.shooterPitch = shooterPitch;
    // this.shooterYaw = shooterYaw;

    // this.drivetrain = drivetrain;
    // this.intake = intake;

    // this.maxCounterValue = period;

    this.operatorController = operatorController;

    addRequirements(shooterFlywheel, shooterPitch/*, shooterYaw */);
  }

  private static double applyDeadband(double joy, double deadband) {
    return Math.abs(joy) < deadband ? 0 : joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelSpeed = 0.0;
  }

  // Track previous button states for edge detection
  private boolean prevPovUp = false;
  private boolean prevPovDown = false;
  private boolean prevPovRight = false;
  private boolean prevPovLeft = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // --- Flywheel speed control (edge detection) ---
    boolean povUp    = operatorController.getHID().getPOV() == 0;
    boolean povDown  = operatorController.getHID().getPOV() == 180;
    boolean povRight = operatorController.getHID().getPOV() == 90;
    boolean povLeft  = operatorController.getHID().getPOV() == 270;

    if (povUp && !prevPovUp) {
        flywheelSpeed += FLYWHEEL_SPEED_INCREMENT;
        shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }
    if (povDown && !prevPovDown) {
        flywheelSpeed -= FLYWHEEL_SPEED_INCREMENT;
        shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }
    if (povRight && !prevPovRight) {
        flywheelSpeed = RIGHT_FLYWHEEL_SPEED;
        shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }
    if (povLeft && !prevPovLeft) {
        flywheelSpeed = LEFT_FLYWHEEL_SPEED;
        shooterFlywheel.setAngularVelocity(flywheelSpeed);
    }

    prevPovUp    = povUp;
    prevPovDown  = povDown;
    prevPovRight = povRight;
    prevPovLeft  = povLeft;

    // // --- Yaw control ---
    // double yawMagnitude = Math.hypot(operatorController.getRightX(), operatorController.getRightY());
    // double yawAngle = (Math.toDegrees(Math.atan2(operatorController.getRightY(), operatorController.getRightX())) % 360 + 360) % 360;
    // if (yawMagnitude >= DEADBAND) {
    //     shooterYaw.setAngularPosition(yawAngle);
    // }

    // --- Pitch control ---
    double pitchMagnitude = Math.hypot(operatorController.getLeftX(), operatorController.getLeftY());
    double pitchAngle = (Math.toDegrees(Math.atan2(operatorController.getLeftY(), operatorController.getLeftX())) % 360 + 360) % 360;
    if (pitchMagnitude >= DEADBAND) {
        shooterPitch.setAngularPosition(pitchAngle);
    }

    // if(RobotBase.isSimulation()) {
    //   // Simulate shooting
    //   counter += 20;
    //   if(counter >= maxCounterValue) {
    //     counter = 0;
    //     intake.simRetrieveBallFromHopper();
    //     RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
    //       // Specify the position of the chassis when the note is launched
    //       drivetrain.getPose().getTranslation(),
    //       // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
    //       new Translation2d(0, 0),
    //       // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
    //       drivetrain.getVelocities(),
    //       // The shooter facing direction is the same as the robot’s facing direction
    //       new Rotation2d(Rotations.of(drivetrain.getPose().getRotation().getRotations()
    //         // Add the shooter’s rotation
    //         /* + shooterYaw.getAngularPosition() */)),
    //       // Initial height of the flying note
    //       Meters.of(0.45),
    //       // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
    //       MetersPerSecond.of(shooterFlywheel.getAngularVelocity() * 960),
    //       // The angle at which the note is launched
    //       Rotations.of(shooterPitch.getAngularPosition())
    //     );
    //     fuelOnFly
    //       // Set the target center to the Rebbuilt Hub of the current alliance
    //       .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
    //       // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the speaker's "mouth")
    //       .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3));
    //       // Set a callback to run when the fuel hits the target
    //       // .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));
    //     fuelOnFly
    //       // Configure callbacks to visualize the flight trajectory of the projectile
    //       .withProjectileTrajectoryDisplayCallBack(
    //         // Callback for when the fuel will eventually hit the target (if configured)
    //         (pose3ds) -> DogLog.log("Shooter/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
    //         // Callback for when the fuel will eventually miss the target, or if no target is configured
    //         (pose3ds) -> DogLog.log("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
    //       );
    //     fuelOnFly
    //       // Configure the note projectile to become a NoteOnField upon touching the ground
    //       .enableBecomesGamePieceOnFieldAfterTouchGround();
        
    //     // Add the projectile to the simulated arena
    //     SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
