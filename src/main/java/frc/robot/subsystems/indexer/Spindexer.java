package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.base.VoltageControlledBase;

/**
 * Spindexer - spins balls into the vertical tunnel.
 * Uses a Falcon 500 with constant voltage control.
 */
public class Spindexer extends VoltageControlledBase {

  {
    SUBSYSTEM_NAME = "Indexer/Spindexer";

    // Hardware devices - CAN ID (adjust as needed)
    motor = new TalonFX(10);

    // Default voltage for operation
    DEFAULT_VOLTAGE = 6.0;

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

  public Spindexer() {
    initialize();
  }
}
