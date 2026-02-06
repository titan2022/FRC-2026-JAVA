package frc.robot.subsystems.shooter;

import static frc.robot.ToSI.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.base.ArmPivot;

public class ShooterPitch extends ArmPivot {
  {
    SUBSYSTEM_NAME = "ShooterPitch";

    // Hardware devices
    motor = new TalonFX(70);
  
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
    MAX_POSITION = 360 * degree;
    MIN_POSITION = 0 * degree;
    STARTING_POSITION = 90 * degree;

    // Basic motor configuration
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POSITION;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POSITION;
    
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO * 2 * Math.PI; // We want everything to be in radians

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

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 5 * radian/s;
    motorConfig.MotionMagic.MotionMagicAcceleration = 5 * radian/(s*s);

    // TODO - Figure out what supply and stator current limits we want
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  public ShooterPitch() {
    super();
  }
}
