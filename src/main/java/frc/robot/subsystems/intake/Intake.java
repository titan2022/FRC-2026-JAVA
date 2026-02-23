package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.base.VoltageControlledBase;

public class Intake extends VoltageControlledBase {
  // TODO: Set to actual voltage
  private static final double INTAKE_VOLTAGE = 1.0;

  {
    SUBSYSTEM_NAME = "Intake";

    // Hardware
    motor = new TalonFX(70);

    // Base defaults
    DEFAULT_VOLTAGE = INTAKE_VOLTAGE;

    // Motor config
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // TODO - Figure out what supply and stator current limits we want
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  public Intake() {
    initialize();
  }

}
