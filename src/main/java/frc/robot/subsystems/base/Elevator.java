package frc.robot.subsystems.base;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends PositionPIDFBase {
  // The following constants must be provided by your subclass.
  public double DRUM_RADIUS = 0.0254; // metres

  public double CARRIAGE_MASS = 5; // kg

  // These are linear positions!
  public double MIN_LINEAR_POSITION;
  public double MAX_LINEAR_POSITION;
  public double STARTING_LINEAR_POSITION;

  public double MAX_VELOCITY;
  public double MAX_ACCELERATION;

  // The following constants are computed in initialize().
  public double METERS_PER_ROTATION;

  // Simulation
  protected ElevatorSim sim;

  protected ElevatorSimVisualization simVisualization;

  public Elevator() {
  }

  // This method MUST be called at the end of subsystem initializers!
  @Override
  protected void initialize() {
    METERS_PER_ROTATION = (2.0 * Math.PI * DRUM_RADIUS) / GEAR_RATIO;

    MIN_ANGULAR_POSITION = MIN_LINEAR_POSITION / METERS_PER_ROTATION;
    MAX_ANGULAR_POSITION = MAX_LINEAR_POSITION / METERS_PER_ROTATION;
    STARTING_ANGULAR_POSITION = STARTING_LINEAR_POSITION / METERS_PER_ROTATION;

    // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGULAR_POSITION;
    // motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGULAR_POSITION;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY / METERS_PER_ROTATION;
    motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION / METERS_PER_ROTATION;

    super.initialize();

    if(RobotBase.isSimulation()) {
      sim = new ElevatorSim(
        gearbox, // Motor type
        GEAR_RATIO,
        CARRIAGE_MASS,
        DRUM_RADIUS,
        MIN_LINEAR_POSITION,
        MAX_LINEAR_POSITION,
        false,
        STARTING_LINEAR_POSITION
      );
      simVisualization = new ElevatorSimVisualization(this);
    }

    

  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    motor.getSimState().setSupplyVoltage(12.0);
    
    double inputVoltage = motor.getSimState().getMotorVoltage();
    DogLog.log(SUBSYSTEM_NAME + "/Sim Input Voltage", inputVoltage, "V");
    
    sim.setInput(inputVoltage);
    sim.update(0.020);
    
    DogLog.log(SUBSYSTEM_NAME + "/Sim Position Meters", sim.getPositionMeters(), "m");
    DogLog.log(SUBSYSTEM_NAME + "/Sim Velocity", sim.getVelocityMetersPerSecond(), "m/s");

    DogLog.log(SUBSYSTEM_NAME + "/MotionMagicCruiseVelocity", motorConfig.MotionMagic.MotionMagicCruiseVelocity);
    DogLog.log(SUBSYSTEM_NAME + "/MotionMagicAcceleration", motorConfig.MotionMagic.MotionMagicAcceleration);
    
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps())
    );

    double motorPosition = sim.getPositionMeters() / METERS_PER_ROTATION * GEAR_RATIO;
    double motorVelocity = sim.getVelocityMetersPerSecond() / METERS_PER_ROTATION * GEAR_RATIO;
    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  @Override
  public void periodic() {
    super.periodic();

    DogLog.log(SUBSYSTEM_NAME + "/Linear Position", getLinearPosition(), "m");
    DogLog.log(SUBSYSTEM_NAME + "/Linear Position Setpoint", setpoint * METERS_PER_ROTATION, "m");
    DogLog.log(SUBSYSTEM_NAME + "/Linear Velocity", getLinearVelocity(), "m/s");
  }

  /**
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public ElevatorSim getSimulation() {
    return sim;
  }

  /**
   * Get the simulation visualization.
   * @return The simulation visualization
   */
  public ElevatorSimVisualization getSimVisualization() {
    return simVisualization;
  }

  /// Get the linear position in meters.
  public double getLinearPosition() {
    return getAngularPosition() * METERS_PER_ROTATION;
  }

  /// Get the linear velocity in meters.
  public double getLinearVelocity() {
    return getAngularVelocity() * METERS_PER_ROTATION;
  }

  /// Set the linear position in meters.
  public void setLinearPosition(double position) {
    setAngularPosition(position / METERS_PER_ROTATION);
  }

  /// Set the linear position in meters.
  public Command setLinearPositionCommand(double position) {
    return setAngularPositionCommand(position / METERS_PER_ROTATION);
  }
}
