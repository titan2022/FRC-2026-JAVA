package frc.robot.subsystems.base;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base for all subsystems which are a single mechanism controlled by constant voltage.
 * This is simpler than PIDF control and is suitable for mechanisms like indexers.
 */
public class VoltageControlledBase extends SubsystemBase {
  // The following fields must be defined by subclasses.
  public String SUBSYSTEM_NAME = "WARNING NAME NOT SET";

  // Hardware devices
  public TalonFX motor;

  // Configuration
  public TalonFXConfiguration motorConfig;

  // Default voltage for operation
  public double DEFAULT_VOLTAGE = 6.0;

  // Motor controller request
  protected VoltageOut voltageRequest = new VoltageOut(0);

  // Motor controller signals
  protected StatusSignal<Angle> positionSignal;
  protected StatusSignal<AngularVelocity> velocitySignal;
  protected StatusSignal<Voltage> voltageSignal;
  protected StatusSignal<Current> statorCurrentSignal;
  protected StatusSignal<Temperature> temperatureSignal;

  // Current setpoint
  protected double voltageSetpoint = 0;

  /**
   * Creates a new Voltage Controlled Subsystem.
   */
  public VoltageControlledBase() {
  }

  // This method MUST be called at the end of subsystem initializers
  protected void initialize() {
    // Get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);
  }

  /**
   * Update telemetry.
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
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Angular Position", getAngularPosition());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Angular Velocity", getAngularVelocity());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Voltage", getVoltage());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Stator Current", getCurrent());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Temperature", getTemperature());
    SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Voltage Setpoint", getVoltageSetpoint());
  }

  /**
   * Get the current position in rotations.
   * @return Position in rotations
   */
  public double getAngularPosition() {
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

  /**
   * Get the current voltage setpoint.
   * @return Current voltage setpoint
   */
  public double getVoltageSetpoint() {
    return voltageSetpoint;
  }

  /**
   * Set motor voltage.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    voltageSetpoint = voltage;
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Run at default voltage.
   */
  public void forward() {
    setVoltage(DEFAULT_VOLTAGE);
  }

  /**
   * Run in reverse at default voltage.
   */
  public void reverse() {
    setVoltage(-DEFAULT_VOLTAGE);
  }

  /**
   * Stop the motor.
   */
  public void stop() {
    setVoltage(0);
    motor.stopMotor();
  }

  /**
   * Creates a command to run at a specific voltage.
   * @param voltage The voltage to apply
   * @return A command that runs the mechanism at the specified voltage
   */
  public Command setVoltageCommand(double voltage) {
    return runOnce(() -> setVoltage(voltage));
  }

  /**
   * Creates a command to run at default voltage.
   * @return A command that runs the mechanism
   */
  public Command forwardCommand() {
    return runOnce(this::forward).finallyDo(this::stop);
  }

  /**
   * Creates a command to run in reverse.
   * @return A command that reverses the mechanism
   */
  public Command reverseCommand() {
    return runOnce(this::reverse).finallyDo(this::stop);
  }

  /**
   * Creates a command to stop the mechanism.
   * @return A command that stops the mechanism
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}