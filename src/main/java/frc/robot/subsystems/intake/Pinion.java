

package frc.robot.subsystems.intake;

import static frc.robot.ToSI.*;

import org.ironmaple.simulation.IntakeSimulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.base.PositionPIDFBase;

public class Pinion extends PositionPIDFBase {
  {
    SUBSYSTEM_NAME = "Pinion";

    // Hardware devices
    motor = new TalonFX(7, HardwareConstants.rioCanbus); 
  
    // Mechanism constants
    gearbox = DCMotor.getFalcon500(1);
    GEAR_RATIO = 9.760802469; 

    // DRUM_RADIUS = 1.3*in; 
    // CARRIAGE_MASS = 10*lb; 

    // Configuration
    // MAX_LINEAR_POSITION = 12.594*in; 
    // MIN_LINEAR_POSITION = 0*in; 
    // STARTING_LINEAR_POSITION = 12.594*in; 
    // The conversion to angular is done in Elevator.initialize()
    MAX_ANGULAR_POSITION = 15.05;
    MIN_ANGULAR_POSITION = 11.47;
    STARTING_ANGULAR_POSITION = MIN_ANGULAR_POSITION;

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();
    
    // motorConfig.SoftwareLimitSwitch is set in Elevator.initialize()

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Feedforward
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    motorConfig.Slot0.kG = 0.08; // ReCalc 0.08 V
    motorConfig.Slot0.kS = 0.0; // TODO 
    motorConfig.Slot0.kV = 5.31; // ReCalc 5.31 V*s/m
    motorConfig.Slot0.kA = 0.04; // ReCalc 0.04 V*s^2/m

    // PID
    motorConfig.Slot0.kP = 149.25; // ReCalc 149.25 V/m
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 6.81; // ReCalc 6.81 V*s/m

    // MAX_VELOCITY = 0.5 * m/s; // ReCalc 2.25 m/s
    // MAX_ACCELERATION = 1.0 * m/(s*s); // ReCalc 21.51 m/s^2
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4; // rot/s
    motorConfig.MotionMagic.MotionMagicAcceleration = 1; // rot/s^2

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 20;

    // https://www.reca.lc/linear?angle=%7B%22s%22%3A11.311%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A20%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A10%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A9.760802469%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A2.6%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A12.594%2C%22u%22%3A%22in%22%7D
  }

  public Pinion() {
    initialize();
  }

  public Command extendIntakeCommand() {
    return setAngularPositionCommand(MAX_ANGULAR_POSITION);
  }

  public Command retractIntakeCommand() {
    return setAngularPositionCommand(MIN_ANGULAR_POSITION);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}


