package frc.robot.drive.ctre.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class DrivingCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private CTRESwerveDrivetrain drivetrain;
  private CommandXboxController driveController;

  private double translationSpeedMultiplier = 1.0;
  private double rotationSpeedMultiplier = 1.0;
  // private double sideMultiplier = Constants.getColor() == Alliance.Blue ? 1.0 : -1.0;
  private double sideMultiplier = 1.0;

  private final SwerveRequest.RobotCentric robotCentricDriveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DEADBAND).withRotationalDeadband(DriverConstants.MAX_ANGULAR_SPEED * DriverConstants.DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
			.withSteerRequestType(SteerRequestType.Position);

  private final SwerveRequest.FieldCentric fieldCentricDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DEADBAND).withRotationalDeadband(DriverConstants.MAX_ANGULAR_SPEED * DriverConstants.DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
			.withSteerRequestType(SteerRequestType.Position);

  private boolean isFieldOriented = true;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric robotCentricStrafe = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentric fieldCentricStrafe = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public DrivingCommand(CTRESwerveDrivetrain drivetrain, CommandXboxController driveController) {
    this.drivetrain = drivetrain;
    this.driveController = driveController;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // sideMultiplier = Constants.getColor() == Alliance.Blue ? 1.0 : -1.0;
    // See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java#L53

    // If you modify these controls please update the diagram at https://docs.google.com/drawings/d/1UsU1iyQz4MPWa87oXD0FYGqLXIfGtkn2a595sXWU3uo/edit.

    // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driveController.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
    // ));

    // Dpad buttons
    // driveController.pov(0).whileTrue(drivetrain.applyRequest(() ->
    //     (isFieldOriented 
    //       ? fieldCentricStrafe.withVelocityX(DriverConstants.DPAD_STRAFE_SPEED * sideMultiplier).withVelocityY(0)
    //       : robotCentricStrafe.withVelocityX(DriverConstants.DPAD_STRAFE_SPEED).withVelocityY(0))
    // ));
    // driveController.pov(90).whileTrue(drivetrain.applyRequest(() ->
    //     (isFieldOriented 
    //       ? fieldCentricStrafe.withVelocityX(0).withVelocityY(-DriverConstants.DPAD_STRAFE_SPEED * sideMultiplier)
    //       : robotCentricStrafe.withVelocityX(0).withVelocityY(-DriverConstants.DPAD_STRAFE_SPEED))
    // ));
    // driveController.pov(180).whileTrue(drivetrain.applyRequest(() ->
    //     (isFieldOriented 
    //       ? fieldCentricStrafe.withVelocityX(-DriverConstants.DPAD_STRAFE_SPEED * sideMultiplier).withVelocityY(0)
    //       : robotCentricStrafe.withVelocityX(-DriverConstants.DPAD_STRAFE_SPEED).withVelocityY(0))
    // ));
    // driveController.pov(270).whileTrue(drivetrain.applyRequest(() ->
    //     (isFieldOriented 
    //       ? fieldCentricStrafe.withVelocityX(0).withVelocityY(DriverConstants.DPAD_STRAFE_SPEED * sideMultiplier)
    //       : robotCentricStrafe.withVelocityX(0).withVelocityY(DriverConstants.DPAD_STRAFE_SPEED))
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

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driveController.y().onTrue(drivetrain.runOnce(() -> drivetrain.resetFieldOrientation())); 
    
    // Alignment slow-down
    driveController.rightBumper().whileTrue(drivetrain.runOnce(() -> {translationSpeedMultiplier = 0.18; rotationSpeedMultiplier = 0.4;}));
    driveController.rightBumper().whileFalse(drivetrain.runOnce(() -> {translationSpeedMultiplier = 1.0; rotationSpeedMultiplier = 1.0;}));

    // field-oriented
    driveController.back().onTrue(drivetrain.runOnce(() -> isFieldOriented = true));
    // robot-oriented
    driveController.start().onTrue(drivetrain.runOnce(() -> isFieldOriented = false));

    // Quick back up motion for dealgifier
    driveController.a().onTrue(drivetrain.applyRequest(() ->
      robotCentricStrafe.withVelocityX(-5.0).withVelocityY(0)
    ).withTimeout(0.1));

    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isFieldOriented) {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setControl(
              fieldCentricDriveRequest
                  .withVelocityX(-driveController.getLeftY() * DriverConstants.MAX_SPEED * translationSpeedMultiplier * sideMultiplier) // Drive forward with negative Y (forward)
                  .withVelocityY(-driveController.getLeftX() * DriverConstants.MAX_SPEED * translationSpeedMultiplier * sideMultiplier) // Drive left with negative X (left)
                  .withRotationalRate(-driveController.getRightX() * DriverConstants.MAX_ANGULAR_SPEED * rotationSpeedMultiplier) // Drive counterclockwise with negative X (left)
      );
    } else {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setControl(
              robotCentricDriveRequest
                  .withVelocityX(-driveController.getLeftY() * DriverConstants.MAX_SPEED * translationSpeedMultiplier) // Drive forward with negative Y (forward)
                  .withVelocityY(-driveController.getLeftX() * DriverConstants.MAX_SPEED * translationSpeedMultiplier) // Drive left with negative X (left)
                  .withRotationalRate(-driveController.getRightX() * DriverConstants.MAX_ANGULAR_SPEED * rotationSpeedMultiplier) // Drive counterclockwise with negative X (left)
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
            robotCentricDriveRequest
                .withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(0) // Drive counterclockwise with negative X (left)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
