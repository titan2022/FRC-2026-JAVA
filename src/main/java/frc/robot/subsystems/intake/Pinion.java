

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
import frc.robot.subsystems.base.Elevator;

public class Pinion extends Elevator {
  private Intake intake;

  {
    SUBSYSTEM_NAME = "Pinion";

    // Hardware devices
    motor = new TalonFX(43); 
  
    // Mechanism constants
    gearbox = DCMotor.getFalcon500(1);
    GEAR_RATIO = 15; 

    DRUM_RADIUS = 0.0254*m; 
    CARRIAGE_MASS = 5*kg; 

    // Configuration
    MAX_LINEAR_POSITION = 1*m; 
    MIN_LINEAR_POSITION = 0*m; 
    STARTING_LINEAR_POSITION = 0.5*m; 
    // The conversion to angular is done in Elevator.initialize()

    // Basic motor configuration
    motorConfig = new TalonFXConfiguration();
    
    // motorConfig.SoftwareLimitSwitch is set in Elevator.initialize()

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Feedforward
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    motorConfig.Slot0.kG = 0.0; 
    motorConfig.Slot0.kS = 0.0; 
    motorConfig.Slot0.kV = 0.0; 
    motorConfig.Slot0.kA = 0.0; 

    // PID
    motorConfig.Slot0.kP = 0.0; 
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 1 * m/s;
    motorConfig.MotionMagic.MotionMagicAcceleration = 1 * m/(s*s);

    // TODO - Figure out what supply and stator current limits we want
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
  }

  public Pinion(Intake intake) {
    this.intake = intake;
    initialize();
  }

  public Command extendIntakeCommand() {
    return setLinearPositionCommand(MIN_LINEAR_POSITION)
      .alongWith(new InstantCommand(() -> intake.setIsOut(true)));
  }

  public Command retractIntakeCommand() {
    return setLinearPositionCommand(MAX_LINEAR_POSITION)
      .alongWith(new InstantCommand(() -> intake.setIsOut(false)));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}


