package frc.robot.subsystems.intake;

import static frc.robot.ToSI.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.base.Elevator;

public class IntakePinion extends Elevator {
  // Rotation is 1 because that's what Phoenix 6 expects.
  public static final double rotation = 1;
  public static final double degree = rotation/360;

  {
    SUBSYSTEM_NAME = "ShooterPitch";

    // Hardware devices
    motor = new TalonFX(41);
  
    // Mechanism constants
    gearbox = DCMotor.getFalcon500(1);
    GEAR_RATIO = 15;

    DRUM_RADIUS = 0.0254*m;
    CARRIAGE_MASS = 5*kg;

    // Configuration
    MAX_LINEAR_POSITION = 360 * degree;
    MIN_LINEAR_POSITION = 0 * degree;
    STARTING_LINEAR_POSITION = 90 * degree;
    // The conversion to angular is done in Elevator.initialize()

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();
    
    // motorConfig.SoftwareLimitSwitch is set in Elevator.initialize()

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Feedforward
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    motorConfig.Slot0.kG = 0.0;
    motorConfig.Slot0.kS = 0.0;
    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kA = 0.0;

    // PID
    motorConfig.Slot0.kP = 0.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 5 * rotation/s;
    motorConfig.MotionMagic.MotionMagicAcceleration = 5 * rotation/(s*s);

    // TODO - Figure out what supply and stator current limits we want
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  public IntakePinion() {
    initialize();
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
