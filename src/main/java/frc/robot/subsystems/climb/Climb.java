package frc.robot.subsystems.climb;

import static frc.robot.ToSI.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.base.Elevator;

public class Climb extends Elevator {
  {
    SUBSYSTEM_NAME = "Climb";

    // Hardware devices
    motor = new TalonFX(21); //We will need to change the CAN ID once more of the robot is built
  
    // Mechanism constants
    gearbox = DCMotor.getFalcon500(1);
    GEAR_RATIO = 15; //Wait until DI figures out the gear ratio to change this

    DRUM_RADIUS = 0.0254*m; //We may need to change this, check with DI
    CARRIAGE_MASS = 5*kg; //We may need to change this, check with DI

    // Configuration
    MAX_LINEAR_POSITION = 1*m; //We will do PIDF tuning later, so this will be something to focus on later
    MIN_LINEAR_POSITION = 0*m; //We will do PIDF tuning later, so this will be something to focus on later
    STARTING_LINEAR_POSITION = 0.5*m; //We will do PIDF tuning later, so this will be something to focus on later
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
    motorConfig.Slot0.kG = 0.0; //We will do PIDF tuning later, so this, along with kA, kS, kV, and kP will be something to focus on later
    motorConfig.Slot0.kA = 0.0; 
    motorConfig.Slot0.kS = 0.0; 
    motorConfig.Slot0.kV = 0.0; 

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

  public Climb() {
    initialize();
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
