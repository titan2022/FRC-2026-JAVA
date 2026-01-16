// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.ToSI.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class KitbotFuelSubsystem extends SubsystemBase {
  // Motor controller IDs for Fuel Mechanism motors
	public static final int FEEDER_MOTOR_ID = 6;
	public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

  public static final int INTAKE_LAUNCHER_ENCODER_A_CHANNEL = 1;
  public static final int INTAKE_LAUNCHER_ENCODER_B_CHANNEL = 0;

	// Current limit and nominal voltage for fuel mechanism motors.
	public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
	public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

	// Voltage values for various fuel operations. These values may need to be tuned
	// based on exact robot construction.
	// See the Software Guide for tuning information
	public static final double INTAKING_FEEDER_VOLTAGE = -12;
	// public static final double INTAKING_INTAKE_VOLTAGE = 10;
	public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
	// public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
	public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
	public static final double SPIN_UP_SECONDS = 1;

  public static final double INTAKE_LAUNCHER_ENCODER_COUNTS_PER_REVOLUTION = 8192; // https://www.revrobotics.com/rev-11-1271/
  public static final double LAUNCHER_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

	// Target velocity
  public double INTAKING_INTAKE_VELOCITY = 1; // m/s
	public double LAUNCHING_LAUNCHER_VELOCITY = 3; // m/s

  // The PIDF controller is to control the intake launcher motor

  // PIDF values
  public double kP = 0.0;
  public double kI = 0.0;
  public double kD = 0.0;

  public double kS = 0.0;
  public double kV = 0.0;
  public double kA = 0.0;

  private PIDController pid = new PIDController(kP, kI, kD);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  private double intakeLauncherTargetVelocity = 0.0;

  private final WPI_TalonSRX feederMotor = new WPI_TalonSRX(FEEDER_MOTOR_ID);
  private final WPI_TalonSRX intakeLauncherMotor = new WPI_TalonSRX(INTAKE_LAUNCHER_MOTOR_ID);
  private final Encoder intakeLauncherEncoder = new Encoder(INTAKE_LAUNCHER_ENCODER_A_CHANNEL, INTAKE_LAUNCHER_ENCODER_B_CHANNEL);

  private final DCMotor feederGearbox = DCMotor.getCIM(1);
  private final DCMotor intakeLauncherGearbox = DCMotor.getCIM(1);
  private final DCMotorSim intakeLauncherMechanismSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      intakeLauncherGearbox,
      2.152 * in * in * lb, // https://discord.com/channels/176186766946992128/368993897495527424/1461521331190370335
      1 // TODO
    ),
    intakeLauncherGearbox
  );
  private final TalonSRXSimCollection feederMotorSim = feederMotor.getSimCollection();
  private final TalonSRXSimCollection intakeLauncherMotorSim = intakeLauncherMotor.getSimCollection();
  private final EncoderSim intakeLauncherEncoderSim = new EncoderSim(intakeLauncherEncoder);

  // MARK - SysId 

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        // Tell SysId how to plumb the driving voltage to the motor(s).
        intakeLauncherMotor::setVoltage,
        // Tell SysId how to record a frame of data for each motor on the mechanism being
        // characterized.
        log -> {
          // Record a frame for the shooter motor.
          log.motor("shooter-wheel")
            .voltage(
              m_appliedVoltage.mut_replace(
                intakeLauncherMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(intakeLauncherEncoder.getDistance() / LAUNCHER_WHEEL_CIRCUMFERENCE, Rotations))
            .angularVelocity(
              m_velocity.mut_replace(intakeLauncherEncoder.getRate() / LAUNCHER_WHEEL_CIRCUMFERENCE, RotationsPerSecond));
        },
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("shooter")
        this));

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /** Creates a new CANBallSubsystem. */
  public KitbotFuelSubsystem() {
    intakeLauncherEncoder.setDistancePerPulse(LAUNCHER_WHEEL_CIRCUMFERENCE / INTAKE_LAUNCHER_ENCODER_COUNTS_PER_REVOLUTION);

    sendValuesToDashboard();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    feederMotor.configPeakCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    intakeLauncherMotor.configPeakCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);

    // intakeLauncherMotor.setInverted(true);
    // intakeLauncherEncoder.setReverseDirection(true);

    // Set the intake launcher setpoint to 0.0
    intakeLauncherTargetVelocity = 0.0;
  }

  public void sendValuesToDashboard() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    SmartDashboard.putData("Intake Launcher/PID", pid);

    SmartDashboard.putNumber("Intake Launcher/kS", kS);
    SmartDashboard.putNumber("Intake Launcher/kV", kV);
    SmartDashboard.putNumber("Intake Launcher/kA", kA);

    SmartDashboard.putNumber("Intake Launcher/Intaking Intake Velocity", INTAKING_INTAKE_VELOCITY);
    SmartDashboard.putNumber("Intake Launcher/Launching Launcher Velocity", LAUNCHING_LAUNCHER_VELOCITY);
  }

  public void getValuesFromDashboard() {
    kS = SmartDashboard.getNumber("Intake Launcher/kS", kS);
    kV = SmartDashboard.getNumber("Intake Launcher/kV", kV);
    kA = SmartDashboard.getNumber("Intake Launcher/kA", kA);

    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);

    INTAKING_INTAKE_VELOCITY = SmartDashboard.getNumber("Intake Launcher/Intaking Intake Velocity", INTAKING_INTAKE_VELOCITY);
    LAUNCHING_LAUNCHER_VELOCITY = SmartDashboard.getNumber("Intake Launcher/Launching Launcher Velocity", LAUNCHING_LAUNCHER_VELOCITY);
  }

  @Override
  public void periodic() {
    getValuesFromDashboard();
    
    double pidfCalculation = pid.calculate(intakeLauncherEncoder.getRate(), intakeLauncherTargetVelocity)
      + feedforward.calculate(intakeLauncherTargetVelocity);
    intakeLauncherMotor.setVoltage(pidfCalculation);

    SmartDashboard.putNumber("Intake Launcher/Target Velocity", intakeLauncherTargetVelocity);
    SmartDashboard.putNumber("Intake Launcher/Current Velocity", intakeLauncherEncoder.getRate());
    SmartDashboard.putNumber("Intake Launcher/Current Voltage", pidfCalculation);
    // SmartDashboard.putNumber("Intake Launcher/Setpoint", pid.getSetpoint()); // actually a velocity
  }

  @Override
  public void simulationPeriodic () {
    intakeLauncherMechanismSim.setInputVoltage(intakeLauncherMotorSim.getMotorOutputLeadVoltage());
    intakeLauncherMechanismSim.update(0.020);

    intakeLauncherEncoderSim.setDistance(intakeLauncherMechanismSim.getAngularPositionRotations() * LAUNCHER_WHEEL_CIRCUMFERENCE);
    intakeLauncherEncoderSim.setRate((intakeLauncherMechanismSim.getAngularVelocityRadPerSec() / (2 * Math.PI)) * LAUNCHER_WHEEL_CIRCUMFERENCE);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederMotor.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherTargetVelocity = INTAKING_INTAKE_VELOCITY;
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederMotor
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherTargetVelocity = -INTAKING_INTAKE_VELOCITY;
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederMotor.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherTargetVelocity = LAUNCHING_LAUNCHER_VELOCITY;
  }

  // A method to stop the rollers
  public void stop() {
    feederMotor.set(0);
    intakeLauncherTargetVelocity = 0.0;
    intakeLauncherMotor.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederMotor
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherTargetVelocity = LAUNCHING_LAUNCHER_VELOCITY;
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }
}
