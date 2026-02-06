package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Both of these need to be replaced with proper voltage values

  private static final double INTAKE_VOLTAGE = 1.0;

  // Use better ID values?
  private static final TalonFX intakeMotor = new TalonFX(70, "rio");

  public Intake() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
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

}
