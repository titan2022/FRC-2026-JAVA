package frc.robot.subsystems.shooter;

import static frc.robot.ToSI.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.base.ArmPivot;

public class ShooterPitch extends ArmPivot {
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

    // If it's a pivot, use the example values below.
    // If it's an arm, get values from the CAD.
    IS_ARM = true;
    ARM_LENGTH = 0.1 * m;
    ARM_MOI = 0.01 * kg*m*m;
    // You can estimate it using SingleJointedArmSim.estimateMOI(armLength, 5).

    // Configuration
    MAX_ANGULAR_POSITION = 360 * degree;
    MIN_ANGULAR_POSITION = 0 * degree;
    STARTING_ANGULAR_POSITION = 90 * degree;

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGULAR_POSITION;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGULAR_POSITION;
    
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

  public ShooterPitch() {
    initialize();
  }

  @Override
  public void periodic() {
    super.periodic();

    DogLog.log(SUBSYSTEM_NAME + "/Position (degrees)", getAngularPosition() / degree, "°");
    DogLog.log(SUBSYSTEM_NAME + "/Position setpoint (degrees)", getSetpoint() / degree, "°");
  }
  public void setTargetDegrees(double degrees) {
    setAngularPosition(degrees * degree);
  }

  public double getAngleDegrees() {
    return getAngularPosition() / degree;
  }

  public double getTargetDegrees() {
    return getSetpoint() / degree;
  }

  public boolean atTargetDegrees(double toleranceDegrees) {
    return Math.abs(getAngleDegrees() - getTargetDegrees()) <= toleranceDegrees;
  }
}
