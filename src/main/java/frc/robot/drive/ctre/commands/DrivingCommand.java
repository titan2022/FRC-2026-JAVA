package frc.robot.drive.ctre.commands;

import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.localization.Vision;
import frc.robot.localization.VisionConstants;


public class DrivingCommand extends Command {
  private CTRESwerveDrivetrain drivetrain;
  private CommandXboxController driveController;

  private double translationSpeedMultiplier = 1.0;
  private double rotationSpeedMultiplier = 1.0;
  private double sideMultiplier = 1.0;

  @SuppressWarnings("unused")
  private DoubleSubscriber kP_subscriber;
  @SuppressWarnings("unused")
  private DoubleSubscriber kI_subscriber;
  @SuppressWarnings("unused")
  private DoubleSubscriber kD_subscriber;

  // --- Hub facing ---
  private final PIDController headingPID = new PIDController(5.0, 0.0, 0.2);
  private boolean isFacingHub = false;

  private final SwerveRequest.RobotCentric robotCentricDriveRequest = new SwerveRequest.RobotCentric()
    .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DEADBAND)
    .withRotationalDeadband(DriverConstants.MAX_ANGULAR_SPEED * DriverConstants.DEADBAND)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position);

  private final SwerveRequest.FieldCentric fieldCentricDriveRequest = new SwerveRequest.FieldCentric()
    .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DEADBAND)
    .withRotationalDeadband(DriverConstants.MAX_ANGULAR_SPEED * DriverConstants.DEADBAND)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position);

  private boolean isFieldOriented = true;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric robotCentricStrafe = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentric fieldCentricStrafe = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public DrivingCommand(CTRESwerveDrivetrain drivetrain,
                        CommandXboxController driveController) {
    this.drivetrain = drivetrain;
    this.driveController = driveController;

    this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
    this.headingPID.setTolerance(Math.toRadians(2.0));

    kP_subscriber = DogLog.tunable(
      "DrivingCommand-Heading/kP", headingPID.getP(), headingPID::setP);
    kI_subscriber = DogLog.tunable(
      "DrivingCommand-Heading/kI", headingPID.getI(), headingPID::setI);
    kD_subscriber = DogLog.tunable(
      "DrivingCommand-Heading/kD", headingPID.getD(), headingPID::setD);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java#L53

    // If you modify these controls please update the diagram at.:
    //   current state: https://docs.google.com/drawings/d/1pWFRHQ_LvV1BaGqvC6eCPU6FR4Ib2r_eUCif9hY9p2g/edit
    //            plan: https://docs.google.com/drawings/d/1ddFERjVCPY4uz2_CGQ1XGQmVzVXk6fk7H-DOadl-NBA/edit

    // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driveController.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
    // ));

    driveController.pov(0).whileTrue(drivetrain.applyRequest(() ->
      robotCentricStrafe.withVelocityX(DriverConstants.DPAD_STRAFE_SPEED).withVelocityY(0)
    ));
    driveController.pov(90).whileTrue(drivetrain.applyRequest(() ->
      robotCentricStrafe.withVelocityX(0).withVelocityY(-DriverConstants.DPAD_STRAFE_SPEED)
    ));
    driveController.pov(180).whileTrue(drivetrain.applyRequest(() ->
      robotCentricStrafe.withVelocityX(-DriverConstants.DPAD_STRAFE_SPEED).withVelocityY(0)
    ));
    driveController.pov(270).whileTrue(drivetrain.applyRequest(() ->
      robotCentricStrafe.withVelocityX(0).withVelocityY(DriverConstants.DPAD_STRAFE_SPEED)
    ));

    // reset the field-centric heading on left bumper press
    driveController.y().onTrue(drivetrain.runOnce(() -> drivetrain.resetFieldOrientation())); 

    // Enable/disable locking onto the HUB
    driveController.leftTrigger().onTrue(drivetrain.runOnce(() -> {
      headingPID.reset();
      isFacingHub = true;
    }));
    driveController.leftBumper().onTrue(drivetrain.runOnce(() -> isFacingHub = false));
    
    // Alignment slow-down
    driveController.rightTrigger().onTrue(drivetrain.runOnce(() -> {translationSpeedMultiplier = 0.18; rotationSpeedMultiplier = 0.4;}));
    driveController.rightBumper().onFalse(drivetrain.runOnce(() -> {translationSpeedMultiplier = 1.0; rotationSpeedMultiplier = 1.0;}));

    // field-oriented
    driveController.back().onTrue(drivetrain.runOnce(() -> isFieldOriented = true));
    driveController.start().onTrue(drivetrain.runOnce(() -> isFieldOriented = false));

    // Quick back up motion for dealgifier
    // driveController.a().onTrue(drivetrain.applyRequest(() ->
    //   robotCentricStrafe.withVelocityX(-5.0).withVelocityY(0)
    // ).withTimeout(0.1));
  }

  public void resetAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      sideMultiplier = alliance.get() == Alliance.Blue ? 1.0 : -1.0;
    } else {
      sideMultiplier = 1.0; // Default is 1.0
    }
  }

  private double getHubRotationOutput() {
    Pose2d robotPose = drivetrain.getState().Pose;

    Translation2d hub = VisionConstants.getHubPosition();

    Translation2d toHub = hub.minus(robotPose.getTranslation());
    double targetAngleRad = Math.atan2(toHub.getY(), toHub.getX()) + ShootingConstants.yaw.in(Radians);
    double currentAngleRad = robotPose.getRotation().getRadians();

    return headingPID.calculate(currentAngleRad, targetAngleRad);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationalRate;
    if (isFacingHub) {
      rotationalRate = getHubRotationOutput();
    } else {
      rotationalRate = -driveController.getRightX()
        * DriverConstants.MAX_ANGULAR_SPEED
        * rotationSpeedMultiplier;
    }

    if (isFieldOriented) {
      drivetrain.setControl(
        fieldCentricDriveRequest
          .withVelocityX(-driveController.getLeftY() * DriverConstants.MAX_SPEED * translationSpeedMultiplier * sideMultiplier)
          .withVelocityY(-driveController.getLeftX() * DriverConstants.MAX_SPEED * translationSpeedMultiplier * sideMultiplier)
          .withRotationalRate(rotationalRate)
      );
    } else {
      drivetrain.setControl(
        robotCentricDriveRequest
          .withVelocityX(-driveController.getLeftY() * DriverConstants.MAX_SPEED * translationSpeedMultiplier)
          .withVelocityY(-driveController.getLeftX() * DriverConstants.MAX_SPEED * translationSpeedMultiplier)
          .withRotationalRate(rotationalRate)
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFacingHub = false;
    drivetrain.setControl(
      robotCentricDriveRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}