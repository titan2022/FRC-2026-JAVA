package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Both of these need to be replaced with proper voltage values

  private static final double INTAKE_VOLTAGE = 1.0;
  private static final double PINION_VOLTAGE = 1.0;

  // Use better ID values?
  private static final TalonFX pinionMotor = new TalonFX(60, "rio");
  private static final TalonFX intakeMotor = new TalonFX(70, "rio");

  public Intake() {
    pinionMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void extendPinion() {
    pinionMotor.setVoltage(PINION_VOLTAGE);
  }

  public void retractPinion() {
    pinionMotor.setVoltage(-PINION_VOLTAGE);
  }

  public void stopPinion() {
    pinionMotor.stopMotor();
  }

  public void forwardIntake() {
    intakeMotor.setVoltage(INTAKE_VOLTAGE);
  }

  public void reverseIntake() {
    intakeMotor.setVoltage(-INTAKE_VOLTAGE);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // Might be worth converting the below to lambda form

  // Command to intake
  public class forwardIntakeCommand extends Command {
    private final Intake intake;

    public forwardIntakeCommand(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.forwardIntake();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stopIntake();
    }
  }

  public Command forwardIntakeCommand() {
    return new forwardIntakeCommand(this);
  }

  // Command to reverse intake
  public class reverseIntakeCommand extends Command {
    private final Intake intake;

    public reverseIntakeCommand(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.reverseIntake();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stopIntake();
    }
  }

  

  // Command to extend pinion
  public class extendPinionCommand extends Command {
    private final Intake intake;

    public extendPinionCommand(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.extendPinion();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stopPinion();
    }

  }

  public Command extendPinionCommand() {
    return new extendPinionCommand(this);
  }

  // Command to retract pinion
  public class retractPinionCommand extends Command {
    private final Intake intake;

    public retractPinionCommand(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.retractPinion();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stopPinion();
    }

  }

  public Command retractPinionCommand() {
    return new retractPinionCommand(this);
  }

}
