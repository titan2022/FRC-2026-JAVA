package frc.robot.subsystems.base;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base for all subsystems which are a single mechanism controlled by position PIDF.
 */
public class VelocityPIDFBase extends SubsystemBase {
// The following fields must be defined by subclasses.
  public String SUBSYSTEM_NAME = "WARNING NAME NOT SET";

  // Hardware devices
  public TalonFX motor;
  
  // Mechanism constants
  public DCMotor gearbox;
  public double GEAR_RATIO;

  // Configuration
  public double STARTING_ANGULAR_POSITION;

  public TalonFXConfiguration motorConfig;

// The following fields are part of this class.

  // Tunable PIDF and profile
  // These fields are initialized in initialize()
  protected DoubleSubscriber kG_subscriber;
  protected DoubleSubscriber kS_subscriber;
  protected DoubleSubscriber kV_subscriber;
  protected DoubleSubscriber kA_subscriber;
  protected DoubleSubscriber kP_subscriber;
  protected DoubleSubscriber kI_subscriber;
  protected DoubleSubscriber kD_subscriber;
  protected DoubleSubscriber maxJerk_subscriber;
  protected DoubleSubscriber maxAcceleration_subscriber;
  protected DoubleSubscriber setpoint_subscriber;

  // Motor controller requests
  protected PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  protected VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  protected MotionMagicVelocityVoltage motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

  // Motor controller signals
  protected StatusSignal<Angle> positionSignal;
  protected StatusSignal<AngularVelocity> velocitySignal;
  protected StatusSignal<Voltage> voltageSignal;
  protected StatusSignal<Current> statorCurrentSignal;
  protected StatusSignal<Temperature> temperatureSignal;

  // Setpoint
  protected double setpoint;

  /**
   * Creates a new Pivot Subsystem.
   */
  public VelocityPIDFBase() {
  }

  // This method MUST be called at the end of subsystem initializers!
  protected void initialize() {
    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);

    // Reset encoder position
    motor.setPosition(STARTING_ANGULAR_POSITION);

    kG_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kG", motorConfig.Slot0.kG, this::configureFromTunable);
    kS_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kS", motorConfig.Slot0.kS, this::configureFromTunable);
    kV_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kV", motorConfig.Slot0.kV, this::configureFromTunable);
    kA_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kA", motorConfig.Slot0.kA, this::configureFromTunable);
    kP_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kP", motorConfig.Slot0.kP, this::configureFromTunable);
    kI_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kI", motorConfig.Slot0.kI, this::configureFromTunable);
    kD_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/kD", motorConfig.Slot0.kD, this::configureFromTunable);
    maxAcceleration_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/max acceleration", motorConfig.MotionMagic.MotionMagicAcceleration, this::configureFromTunable);
    maxJerk_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/max jerk", motorConfig.MotionMagic.MotionMagicJerk, this::configureFromTunable);
    setpoint_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/angular velocity setpoint", setpoint, this::setAngularVelocity);
  }

  public void configureFromTunable(double unused) {
    motorConfig.Slot0.kG = kG_subscriber.get();
    motorConfig.Slot0.kS = kS_subscriber.get();
    motorConfig.Slot0.kV = kV_subscriber.get();
    motorConfig.Slot0.kA = kA_subscriber.get();
    motorConfig.Slot0.kP = kP_subscriber.get();
    motorConfig.Slot0.kI = kI_subscriber.get();
    motorConfig.Slot0.kD = kD_subscriber.get();

    motorConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration_subscriber.get();
    motorConfig.MotionMagic.MotionMagicJerk = maxJerk_subscriber.get();

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
    DogLog.log(SUBSYSTEM_NAME + "/Angular Position", getAngularPosition(), "rotation");
    DogLog.log(SUBSYSTEM_NAME + "/Angular Velocity", getAngularVelocity(), "rotation/s");
    DogLog.log(SUBSYSTEM_NAME + "/Voltage", getVoltage(), "V");
    DogLog.log(SUBSYSTEM_NAME + "/Stator Current", getCurrent(), "A");
    DogLog.log(SUBSYSTEM_NAME + "/Temperature", getTemperature(), "°C");
    // DogLog.log(SUBSYSTEM_NAME + "/Velocity setpoint", getSetpoint(), "rotation/s");
  }

  /**
   * Get the current position in rotations.
   * @return Position in rotations
   */
  public double getAngularPosition() {
    // Rotations
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  public double getAngularVelocity() {
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

  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Stop the motor.
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Sets the PIDF setpoint to a specific velocity.
   * Motion Magic is used to make a trapezoidal profile and apply a PIDF controller.
   * @param velocity The target velocity in rotations per second
   */
  public void setAngularVelocity(double velocity) {
    setpoint = velocity;
    motor.setControl(motionRequest.withVelocity(velocity));
  }

  /**
   * Creates a command to set the PIDF setpoint to a specific velocity.
   * Motion Magic is used to make a trapezoidal profile and apply a PIDF controller.
   * @param velocity The target velocity in rotations per second
   * @return A command that sets the PIDF setpoint to the specified velocity
   */
  public Command setAngularPositionCommand(double velocity) {
    return runOnce(() -> setAngularVelocity(velocity));
  }

  /**
   * Creates a command to stop the mechanism.
   * @return A command that stops the mechanism
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
