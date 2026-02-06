package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Example of an arm or a pivot
 */
public class ArmPivot extends SubsystemBase {
  public static final String SUBSYSTEM_NAME = "ArmPivot";

  // Hardware devices
  private final TalonFX motor = new TalonFX(70);
  
  // Mechanism constants
  private static final DCMotor gearbox = DCMotor.getFalcon500(1);
  private static final double GEAR_RATIO = 15;

  // If it's a pivot, use the example values below. Otherwise change it to match the reality.
  public static final double ARM_LENGTH = 0.1 * m;
  public static final double ARM_MOI = 0.01 * kg*m*m;
  // You can estimate it using SingleJointedArmSim.estimateMOI(armLength, 5).

  // Configuration
  private static final double MAX_POSITION = 360 * degree;
  private static final double MIN_POSITION = 0 * degree;
  private static final double STARTING_POSITION = 0 * degree;

  private static TalonFXConfiguration config = new TalonFXConfiguration();
  static {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POSITION;
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO * 2 * Math.PI; // We want everything to be in radians

    // Feedforward
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.kG = 0.0; // There is no gravity on the yaw
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;

    // PID
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicCruiseVelocity = 5 * radian/s;
    config.MotionMagic.MotionMagicAcceleration = 5 * radian/(s*s);

    // TODO - Figure out what supply and stator current limits we want
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  // Tunable PIDF and profile
  private final DoubleSubscriber kG_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kG", config.Slot0.kG, this::configureFromTunable);
  private final DoubleSubscriber kS_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kS", config.Slot0.kS, this::configureFromTunable);
  private final DoubleSubscriber kV_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kV", config.Slot0.kV, this::configureFromTunable);
  private final DoubleSubscriber kA_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kA", config.Slot0.kA, this::configureFromTunable);
  private final DoubleSubscriber kP_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kP", config.Slot0.kP, this::configureFromTunable);
  private final DoubleSubscriber kI_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kI", config.Slot0.kI, this::configureFromTunable);
  private final DoubleSubscriber kD_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/kD", config.Slot0.kD, this::configureFromTunable);
  private final DoubleSubscriber maxVelocity_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/max velocity", config.MotionMagic.MotionMagicCruiseVelocity, this::configureFromTunable);
    private final DoubleSubscriber maxAcceleration_subscriber = DogLog.tunable(
    SUBSYSTEM_NAME + "/max acceleration", config.MotionMagic.MotionMagicAcceleration, this::configureFromTunable);

  // Motor controller requests
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0).withSlot(0);

  // Motor controller signals
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
    gearbox, // Motor type
    GEAR_RATIO,
    ARM_MOI, // Arm moment of inertia - Small value since there are no arm parameters
    ARM_LENGTH, // Arm length (m) - Small value since there are no arm parameters
    MIN_POSITION, // Min angle (rad)
    MAX_POSITION, // Max angle (rad)
    false, // Simulate gravity - Disable gravity for pivot
    STARTING_POSITION // Starting position (rad)
  );

  /**
   * Creates a new Pivot Subsystem.
   */
  public ArmPivot() {
    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Reset encoder position
    motor.setPosition(STARTING_POSITION);
  }

  public void configureFromTunable(double unused) {
    config.Slot0.kG = kG_subscriber.get();
    config.Slot0.kS = kS_subscriber.get();
    config.Slot0.kV = kV_subscriber.get();
    config.Slot0.kA = kA_subscriber.get();
    config.Slot0.kP = kP_subscriber.get();
    config.Slot0.kI = kI_subscriber.get();
    config.Slot0.kD = kD_subscriber.get();

    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity_subscriber.get();
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration_subscriber.get();

    // Apply configuration
    motor.getConfigurator().apply(config);
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
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()), pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use motor voltage for TalonFX simulation input
    pivotSim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        pivotSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = Radians.of(pivotSim.getAngleRads() * GEAR_RATIO).in(
      Rotations
    );
    double motorVelocity = RadiansPerSecond.of(
      pivotSim.getVelocityRadPerSec() * GEAR_RATIO
    ).in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
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
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
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
