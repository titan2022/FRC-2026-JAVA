package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.base.VoltageControlledBase;

/**
 * Vertical Indexer - kicker + vertical tunnel rollers.
 * Uses a Falcon 500 with constant voltage control.
 */
public class VerticalIndexer extends VoltageControlledBase {

  {
    SUBSYSTEM_NAME = "Indexer/VerticalIndexer";

    // Hardware devices - CAN ID (adjust to the real value)
    motor = new TalonFX(11);

    // Default voltage for operation
    DEFAULT_VOLTAGE = 8.0;

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();
    
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  public VerticalIndexer() {
    initialize();
  }
}
