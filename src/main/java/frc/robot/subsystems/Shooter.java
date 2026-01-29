package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  private static final double INTAKE_VOLTAGE = 1.0;
  private static final double PINION_VOLTAGE = 1.0;

  private static final TalonFX turretMotor = new TalonFX(30, "rio");
  private static final TalonFX pitchMotor = new TalonFX(31, "rio");
  private static final TalonFX flywheelMotor = new TalonFX(31, "rio");

  private boolean isIntaking = false;
  private boolean isExtending = false;
  
  public Shooter() {
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
    pitchMotor.setNeutralMode(NeutralModeValue.Brake);
    flywheelMotor.setNeutralMode(NeutralModeValue.Brake);
  }

}
