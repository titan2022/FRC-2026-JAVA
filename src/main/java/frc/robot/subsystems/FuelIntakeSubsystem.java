package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FuelIntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_VOLTAGE = 1.0;
  private static final double PINION_VOLTAGE = 1.0;

  private static final TalonFX pinionMotor = new TalonFX(60, "rio");
  private static final TalonFX intakeMotor = new TalonFX(70, "rio");

  private boolean isIntaking = false;
  private boolean isExtending = false;
  
  public FuelIntakeSubsystem() {
    pinionMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void runPinion() {
    pinionMotor.setVoltage(PINION_VOLTAGE);
    isExtending = true;
  }

  public void stopPinion() {
    pinionMotor.stopMotor();
    isExtending = false;
  }

  public void runIntake() {
    intakeMotor.setVoltage(INTAKE_VOLTAGE);
    isIntaking = true;
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    isIntaking = false;
  }

  // Add function to check intake, pinion status

}
