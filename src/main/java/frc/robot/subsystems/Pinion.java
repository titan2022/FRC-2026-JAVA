package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pinion extends SubsystemBase {

  // Both of these need to be replaced with proper voltage values

  private static final double PINION_VOLTAGE = 1.0;

  // Use better ID values?
  private static final TalonFX pinionMotor = new TalonFX(60, "rio");

  public Pinion() {
    pinionMotor.setNeutralMode(NeutralModeValue.Brake);
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


  // Might be worth converting the below to lambda form

  // Command to extend pinion
  public class extendPinionCommand extends Command {
    private final Pinion pinion;

    public extendPinionCommand(Pinion pinion) {
      this.pinion = pinion;
      addRequirements(pinion);
    }

    @Override
    public void initialize() {
      pinion.extendPinion();
    }

    @Override
    public void end(boolean interrupted) {
      pinion.stopPinion();
    }

  }

  public Command extendPinionCommand() {
    return new extendPinionCommand(this);
  }

  // Command to retract pinion
  public class retractPinionCommand extends Command {
    private final Pinion pinion;

    public retractPinionCommand(Pinion pinion) {
      this.pinion = pinion;
      addRequirements(pinion);
    }

    @Override
    public void initialize() {
      pinion.retractPinion();
    }

    @Override
    public void end(boolean interrupted) {
      pinion.stopPinion();
    }

  }

  public Command retractPinionCommand() {
    return new retractPinionCommand(this);
  }

}
