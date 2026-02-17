package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.ToSI.kg;
import static frc.robot.ToSI.m;
import static frc.robot.ToSI.s;
import frc.robot.subsystems.base.Flywheel;

/**
 * Indexer Flywheel - shoots the fuel.
 * Uses a Kraken X60 with velocity PIDF control.
 */
public class IndexerFlywheel extends Flywheel {
  // Rotation is 1 because that's what Phoenix 6 expects.
  public static final double rotation = 1;

  // Target velocity for shooting
  public static final double TARGET_VELOCITY_RPS = 80.0; // rotations per second
  
  // Tolerance for "at speed" check
  public static final double VELOCITY_TOLERANCE_RPS = 2.0;

  {
    SUBSYSTEM_NAME = "IndexerFlywheel";

    // Hardware devices - CAN ID (adjust as needed)
    motor = new TalonFX(12);
  
    // Mechanism constants - Kraken X60
    gearbox = DCMotor.getKrakenX60(1);
    GEAR_RATIO = 1; // Direct drive or adjust as needed

    FLYWHEEL_MOI = 0.01 * kg*m*m;

    // Configuration
    STARTING_ANGULAR_POSITION = 0;

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();
    
    // Flywheel should coast so it doesn't burn out when stopped
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Feedforward
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    motorConfig.Slot0.kG = 0.0; // No gravity compensation for flywheel
    motorConfig.Slot0.kS = 0.1; // Static friction
    motorConfig.Slot0.kV = 0.12; // Feed-forward velocity
    motorConfig.Slot0.kA = 0.0;

    // PID - need to be tuned
    motorConfig.Slot0.kP = 0.5;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    // Motion Magic settings for velocity control
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100 * rotation/s;
    motorConfig.MotionMagic.MotionMagicAcceleration = 200 * rotation/(s*s);

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  public IndexerFlywheel() {
    initialize();
  }

  /**
   * Spin the flywheel at target velocity for shooting.
   */
  public void spin() {
    setAngularVelocity(TARGET_VELOCITY_RPS);
  }

  /**
   * Stop the flywheel.
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Check if flywheel is at target velocity.
   * @return true if within tolerance of target velocity
   */
  public boolean isAtSpeed() {
    double currentVelocity = getAngularVelocity();
    return Math.abs(currentVelocity - TARGET_VELOCITY_RPS) < VELOCITY_TOLERANCE_RPS;
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Target Velocity RPS", TARGET_VELOCITY_RPS);
    SmartDashboard.putBoolean(SUBSYSTEM_NAME + "/At Speed", isAtSpeed());
  }
}
