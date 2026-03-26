package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

public class VoltagePinion extends SubsystemBase {
	public static final String SUBSYSTEM_NAME = "VoltagePinion";
	public final TalonFX motor = new TalonFX(7, HardwareConstants.rioCanbus);

	private final VoltageOut m_voltageControl = new VoltageOut(0);

	public static final double EXTEND_VOLTAGE = 1.2;
	public static final double RETRACT_VOLTAGE = -1.2;

	public static final double CURRENT_THRESHOLD = 16;

	public final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
	{
		motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
	}

	// Motor controller signals
	protected StatusSignal<Angle> positionSignal;
	protected StatusSignal<AngularVelocity> velocitySignal;
	protected StatusSignal<Voltage> voltageSignal;
	protected StatusSignal<Current> statorCurrentSignal;
	protected StatusSignal<Temperature> temperatureSignal;

	public VoltagePinion() {
		// Get status signals
		positionSignal = motor.getPosition();
		velocitySignal = motor.getVelocity();
		voltageSignal = motor.getMotorVoltage();
		statorCurrentSignal = motor.getStatorCurrent();
		temperatureSignal = motor.getDeviceTemp();

		// Apply configuration
		motor.getConfigurator().apply(motorConfig);
	}

	public Command extendIntakeCommand() {
		return runEnd(
			() -> motor.setVoltage(EXTEND_VOLTAGE),
			this::stop
		).until(() -> motor.getStatorCurrent().getValueAsDouble() > CURRENT_THRESHOLD);
	}

	public Command retractIntakeCommand() {
		return runEnd(
			() -> motor.setVoltage(RETRACT_VOLTAGE),
			this::stop
		).until(() -> motor.getStatorCurrent().getValueAsDouble() > CURRENT_THRESHOLD);
	}

	public void stop() {
		motor.stopMotor();
	}

	public Command stopCommand() {
		return runOnce(this::stop);
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
}
