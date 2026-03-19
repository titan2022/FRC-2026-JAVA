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
    // motorConfig.Slot0.kG = 0.0;
    // motorConfig.Slot0.kS = 0.0;
    // motorConfig.Slot0.kV = 0.0;
    // motorConfig.Slot0.kA = 0.0;

    // // PID
    // motorConfig.Slot0.kP = 0.0;
    // motorConfig.Slot0.kI = 0.0;
    // motorConfig.Slot0.kD = 0.0;

    // https://www.reca.lc/linear?angle=%7B"s"%3A11.311%2C"u"%3A"deg"%7D&currentLimit=%7B"s"%3A40%2C"u"%3A"A"%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B"s"%3A400%2C"u"%3A"in%2Fs2"%7D&limitedDeceleration=%7B"s"%3A50%2C"u"%3A"in%2Fs2"%7D&limitedVelocity=%7B"s"%3A10%2C"u"%3A"in%2Fs"%7D&load=%7B"s"%3A10%2C"u"%3A"lbs"%7D&motor=%7B"quantity"%3A1%2C"name"%3A"Falcon%20500"%7D&ratio=%7B"magnitude"%3A9.760802469%2C"ratioType"%3A"Reduction"%7D&spoolDiameter=%7B"s"%3A2.6%2C"u"%3A"in"%7D&travelDistance=%7B"s"%3A12.594%2C"u"%3A"in"%7D
    motorConfig.Slot0.kG = 0.0; // ReCalc 0.00 V
    motorConfig.Slot0.kS = 0.0; // not calculated
    motorConfig.Slot0.kV = 19.50; // ReCalc 19.50 V*s/rot
    motorConfig.Slot0.kA = 0.0; // ReCalc 0.00 V*s^2/rot

    // PID
    motorConfig.Slot0.kP = 0.0; // ReCalc 0.00 V/rot
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0; // ReCalc 0.00 V*s/rot

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
    DogLog.log(SUBSYSTEM_NAME + "/Position setpoint (degrees)", getAngularPositionSetpoint() / degree, "°");
  }
}
