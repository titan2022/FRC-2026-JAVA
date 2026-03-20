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
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.base.ArmPivot;

public class ShooterPitch extends ArmPivot {
  // Rotation is 1 because that's what Phoenix 6 expects.
  public static final double rotation = 1;
  public static final double degree = rotation/360;

  {
    SUBSYSTEM_NAME = "ShooterPitch";

    // Hardware devices
    motor = new TalonFX(45, HardwareConstants.rioCanbus);
  
    // Mechanism constants
    gearbox = DCMotor.getFalcon500(1);
    GEAR_RATIO = 172.8;

    // If it's a pivot, use the example values below.
    // If it's an arm, get values from the CAD.
    IS_ARM = true;
    ARM_LENGTH = 20*cm;
    ARM_MOI = 0.008821156008 * kg*m*m;
    // You can estimate it using SingleJointedArmSim.estimateMOI(armLength, 5).

    // Configuration
    MAX_ANGULAR_POSITION = 50 * degree;
    MIN_ANGULAR_POSITION = 23 * degree;
    STARTING_ANGULAR_POSITION = 23 * degree;

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
    // motorConfig.Slot0.kG = 0.0;
    // motorConfig.Slot0.kS = 0.0;
    // motorConfig.Slot0.kV = 0.0;
    // motorConfig.Slot0.kA = 0.0;

    // // PID
    // motorConfig.Slot0.kP = 0.0;
    // motorConfig.Slot0.kI = 0.0;
    // motorConfig.Slot0.kD = 0.0;

    // https://www.reca.lc/arm?armMass=%7B%22s%22%3A1.3458353%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A17%2C%22u%22%3A%22cm%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A50%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A172.8%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A23%2C%22u%22%3A%22deg%22%7D
    motorConfig.Slot0.kG = 0.02; // ReCalc 0.02 V
    motorConfig.Slot0.kS = 0.0; // not calculated
    motorConfig.Slot0.kV = 19.50; // ReCalc 19.50 V*s/rot
    motorConfig.Slot0.kA = 0.0; // ReCalc 0.00 V*s^2/rot

    // PID
    motorConfig.Slot0.kP = 505.87; // ReCalc 505.87 V/rot
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.04; // ReCalc 0.04 V*s/rot

    // motorConfig.MotionMagic.MotionMagicCruiseVelocity = 5 * rotation/s;
    // motorConfig.MotionMagic.MotionMagicAcceleration = 5 * rotation/(s*s);

    // TODO - Figure out what supply and stator current limits we want
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 40;
  }

  public ShooterPitch() {
    initialize();
  }

  @Override
  public void periodic() {
    super.periodic();

    DogLog.log(SUBSYSTEM_NAME + "/Position (degrees)", getAngularPosition() / degree, "°");
    DogLog.log(SUBSYSTEM_NAME + "/Position setpoint (degrees)", getAngularPositionSetpoint() / degree, "°");
  }
}
