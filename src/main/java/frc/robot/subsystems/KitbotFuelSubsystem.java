// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KitbotFuelSubsystem extends SubsystemBase {
  // Motor controller IDs for Fuel Mechanism motors
	public static final int FEEDER_MOTOR_ID = 6;
	public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

  public static final int INTAKE_LAUNCHER_ENCODER_A_CHANNEL = 1;
  public static final int INTAKE_LAUNCHER_ENCODER_B_CHANNEL = 2;

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
  private double intakeLauncherSetpoint = 0.0;

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final Encoder intakeLauncherEncoder = new Encoder(INTAKE_LAUNCHER_ENCODER_A_CHANNEL, INTAKE_LAUNCHER_ENCODER_B_CHANNEL);

  /** Creates a new CANBallSubsystem. */
  public KitbotFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);

    sendValuesToDashboard();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    // Set the intake launcher setpoint to 0.0
    intakeLauncherSetpoint = 0.0;
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

    SmartDashboard.putData("KitbotFuelSubsystem/PID", pid);

    SmartDashboard.putNumber("KitbotFuelSubsystem/kS", kS);
    SmartDashboard.putNumber("KitbotFuelSubsystem/kV", kV);
    SmartDashboard.putNumber("KitbotFuelSubsystem/kA", kA);

    SmartDashboard.putNumber("KitbotFuelSubsystem/Intaking Intake Velocity", INTAKING_INTAKE_VELOCITY);
    SmartDashboard.putNumber("KitbotFuelSubsystem/Launching Launcher Velocity", LAUNCHING_LAUNCHER_VELOCITY);
  }

  public void getValuesFromDashboard() {
    kS = SmartDashboard.getNumber("KitbotFuelSubsystem/kS", kS);
    kV = SmartDashboard.getNumber("KitbotFuelSubsystem/kV", kV);
    kA = SmartDashboard.getNumber("KitbotFuelSubsystem/kA", kA);

    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);

    INTAKING_INTAKE_VELOCITY = SmartDashboard.getNumber("KitbotFuelSubsystem/Intaking Intake Velocity", INTAKING_INTAKE_VELOCITY);
    LAUNCHING_LAUNCHER_VELOCITY = SmartDashboard.getNumber("KitbotFuelSubsystem/Launching Launcher Velocity", LAUNCHING_LAUNCHER_VELOCITY);
  }

  @Override
  public void periodic() {
    intakeLauncherRoller.setVoltage(pid.calculate(intakeLauncherEncoder.getRate(), intakeLauncherSetpoint) + feedforward.calculate(intakeLauncherSetpoint));
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherSetpoint = INTAKING_INTAKE_VELOCITY;
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherSetpoint = -INTAKING_INTAKE_VELOCITY;
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherSetpoint = LAUNCHING_LAUNCHER_VELOCITY;
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherSetpoint = 0.0;
    intakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherSetpoint = LAUNCHING_LAUNCHER_VELOCITY;
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
