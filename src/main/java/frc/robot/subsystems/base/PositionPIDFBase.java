package frc.robot.subsystems.base;

import static frc.robot.ToSI.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base for all subsystems which are a single mechanism controlled by position PIDF.
 */
public class PositionPIDFBase extends SubsystemBase {
  public String SUBSYSTEM_NAME = "PositionPIDFBase";

  // Hardware devices
  public TalonFX motor = new TalonFX(70);
  
  // Mechanism constants
  public DCMotor gearbox = DCMotor.getFalcon500(1);
  public double GEAR_RATIO = 15;

  // Configuration
  public double MAX_POSITION = 1;
  public double MIN_POSITION = 0;
  public double STARTING_POSITION = 0;

  public TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  {
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

  // Tunable PIDF and profile
  protected DoubleSubscriber kG_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kG", motorConfig.Slot0.kG, this::configureFromTunable);
  protected DoubleSubscriber kS_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kS", motorConfig.Slot0.kS, this::configureFromTunable);
  protected DoubleSubscriber kV_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kV", motorConfig.Slot0.kV, this::configureFromTunable);
  protected DoubleSubscriber kA_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kA", motorConfig.Slot0.kA, this::configureFromTunable);
  protected DoubleSubscriber kP_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kP", motorConfig.Slot0.kP, this::configureFromTunable);
  protected DoubleSubscriber kI_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kI", motorConfig.Slot0.kI, this::configureFromTunable);
  protected DoubleSubscriber kD_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kD", motorConfig.Slot0.kD, this::configureFromTunable);
  protected DoubleSubscriber maxVelocity_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/max velocity", motorConfig.MotionMagic.MotionMagicCruiseVelocity, this::configureFromTunable);
  protected DoubleSubscriber maxAcceleration_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/max acceleration", motorConfig.MotionMagic.MotionMagicAcceleration, this::configureFromTunable);

  // Motor controller requests
  protected PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  protected VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  protected MotionMagicVoltage motionRequest = new MotionMagicVoltage(0).withSlot(0);

  // Motor controller signals
  protected StatusSignal<Angle> positionSignal;
  protected StatusSignal<AngularVelocity> velocitySignal;
  protected StatusSignal<Voltage> voltageSignal;
  protected StatusSignal<Current> statorCurrentSignal;
  protected StatusSignal<Temperature> temperatureSignal;

  /**
   * Creates a new Pivot Subsystem.
   */
  public PositionPIDFBase() {
    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);

    // Reset encoder position
    motor.setPosition(STARTING_POSITION);
  }

  public void configureFromTunable(double unused) {
    motorConfig.Slot0.kG = kG_subscriber.get();
    motorConfig.Slot0.kS = kS_subscriber.get();
    motorConfig.Slot0.kV = kV_subscriber.get();
    motorConfig.Slot0.kA = kA_subscriber.get();
    motorConfig.Slot0.kP = kP_subscriber.get();
    motorConfig.Slot0.kI = kI_subscriber.get();
    motorConfig.Slot0.kD = kD_subscriber.get();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity_subscriber.get();
    motorConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration_subscriber.get();

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      positionSignal,
      velocitySignal,
      voltageSignal,
      statorCurrentSignal,
      temperatureSignal
    );

    // Log values
    DogLog.log(SUBSYSTEM_NAME + "/Position", getPosition(), "rad");
    DogLog.log(SUBSYSTEM_NAME + "/Velocity", getVelocity(), "rad/s");
    DogLog.log(SUBSYSTEM_NAME + "/Voltage", getVoltage(), "V");
    DogLog.log(SUBSYSTEM_NAME + "/Stator Current", getCurrent(), "A");
    DogLog.log(SUBSYSTEM_NAME + "/Temperature", getTemperature(), "°C");
  }

  /**
   * Get the current position in radians.
   * @return Position in radians
   */
  public double getPosition() {
    // Rotations
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in radians per second.
   * @return Velocity in radians per second
   */
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Sets the PIDF setpoint to a specific angle.
   * Motion Magic is used to make a trapezoidal profile and apply a PIDF controller.
   * @param position The target angle in radians
   */
  public void setPosition(double position) {
    motor.setControl(motionRequest.withPosition(position));
  }

  /**
   * Creates a command to set the PIDF setpoint to a specific angle.
   * Motion Magic is used to make a trapezoidal profile and apply a PIDF controller.
   * @param position The target angle in radians
   * @return A command that sets the PIDF setpoint to the specified angle
   */
  public Command setPositionCommand(double position) {
    return runOnce(() -> setPosition(position));
  }

  /**
   * Creates a command to stop the mechanism.
   * @return A command that stops the mechanism
   */
  public Command stopCommand() {
    return runOnce(() -> motor.stopMotor());
  }
}
