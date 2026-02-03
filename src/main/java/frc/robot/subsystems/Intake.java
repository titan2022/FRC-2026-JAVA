package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final double INTAKE_VOLTAGE = 1.0;
  private static final double PINION_VOLTAGE = 1.0;

  private static final TalonFX pinionMotor = new TalonFX(60, "rio");
  private static final TalonFX intakeMotor = new TalonFX(70, "rio");

  private boolean isIntaking = false;
  private boolean isExtending = false;

  public Intake() {
    pinionMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void extendPinion() {
    pinionMotor.setVoltage(PINION_VOLTAGE);
    isExtending = true;
  }

  public void reversePinion() {
    pinionMotor.setVoltage(-PINION_VOLTAGE);
    isExtending = false;
  }

  public void stopPinion() {
    pinionMotor.stopMotor();
    isExtending = false;
  }

  public void runIntake() {
    intakeMotor.setVoltage(INTAKE_VOLTAGE);
    isIntaking = true;
  }

  public void reverseIntake() {
    intakeMotor.setVoltage(-INTAKE_VOLTAGE);
    isIntaking = false;
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    isIntaking = false;
  }

  // Add function to check intake, pinion status
  public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stopIntake();
    }
  }

  public Command intakeCommand() {
    return new IntakeCommand(this);
  }

  // Add function to check reverse intake, pinion status
  public class ReverseIntakeCommand extends Command {
    private final Intake intake;

    public ReverseIntakeCommand(Intake intake) {
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

  public Command reverseIntakeCommand() {
    return new ReverseIntakeCommand(this);
  }

  public class ReversePinionCommand extends Command {
    private final Intake intake;

    public ReversePinionCommand(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.reversePinion();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stopPinion();
    }

  }

  public Command ReversePinionCommand() {
    return new ReversePinionCommand(this);
  }

}
