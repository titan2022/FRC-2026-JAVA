package frc.robot.subsystems.shooter;

import static frc.robot.ToSI.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.base.Flywheel;

public class ShooterFlywheel extends Flywheel {
  // Rotation is 1 because that's what Phoenix 6 expects.
  public static final double rotation = 1;
  public static final double degree = rotation/360;

  // There are three motors in total.
  public TalonFX follower1;
  public TalonFX follower2;

  // The only thing that should change between them is inversion state.
  private MotorAlignmentValue follower1_alignment;
  private MotorAlignmentValue follower2_alignment;

  {
    SUBSYSTEM_NAME = "ShooterFlywheel";

    // Hardware devices
    motor = new TalonFX(44, HardwareConstants.rioCanbus);
    follower1 = new TalonFX(60, HardwareConstants.rioCanbus);
    follower2 = new TalonFX(61, HardwareConstants.rioCanbus);


  
    // Mechanism constants
    gearbox = DCMotor.getFalcon500(1);
    GEAR_RATIO = 15;

    FLYWHEEL_MOI = 0.01 * kg*m*m;

    // Configuration
    STARTING_ANGULAR_POSITION = 90 * degree;

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();
    
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    follower1_alignment = MotorAlignmentValue.Aligned;
    follower2_alignment = MotorAlignmentValue.Aligned;
    
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Feedforward
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    motorConfig.Slot0.kG = 0.0; // Since this is a pivot, kG should always be 0.
    motorConfig.Slot0.kS = 0.0;
    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kA = 0.0;

    // PID
    motorConfig.Slot0.kP = 0.1;
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

  public ShooterFlywheel() {
    initialize();
  }

  @Override
  protected void initialize() {
    super.initialize();

    follower1.setControl(new Follower(motor.getDeviceID(), follower1_alignment));
    follower2.setControl(new Follower(motor.getDeviceID(), follower2_alignment));
  }

  @Override
  public void applyMotorConfig() {
    motor.getConfigurator().apply(motorConfig);
    follower1.getConfigurator().apply(motorConfig);
    follower2.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
    super.periodic();

    DogLog.log(SUBSYSTEM_NAME + "/Position (degrees)", getAngularPosition() / degree, "°");
    DogLog.log(SUBSYSTEM_NAME + "/Position setpoint (degrees)", getSetpoint() / degree, "°");
  }
}
