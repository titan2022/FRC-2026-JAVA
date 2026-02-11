package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Example of an arm or a pivot
 */
public class Elevator extends PositionPIDFBase {
  // The following constants must be provided by your subclass.
  public double DRUM_RADIUS = 0.0254; // metres

  public double CARRIAGE_MASS = 5; // kg

  // These are linear positions!
  public double MIN_LINEAR_POSITION;
  public double MAX_LINEAR_POSITION;
  public double STARTING_LINEAR_POSITION;

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
    super.initialize();

    METERS_PER_ROTATION = (1 / (2.0 * Math.PI * DRUM_RADIUS)) * GEAR_RATIO;

    MIN_ANGULAR_POSITION = MIN_LINEAR_POSITION / METERS_PER_ROTATION;
    MAX_ANGULAR_POSITION = MAX_LINEAR_POSITION / METERS_PER_ROTATION;
    STARTING_ANGULAR_POSITION = STARTING_LINEAR_POSITION / METERS_PER_ROTATION;

    if(RobotBase.isSimulation()) {
      sim = new ElevatorSim(
        gearbox, // Motor type
        GEAR_RATIO,
        CARRIAGE_MASS,
        DRUM_RADIUS,
        MIN_LINEAR_POSITION,
        MAX_LINEAR_POSITION,
        true,
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
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //sim.setInput(dcMotor.getVoltage(dcMotor.getTorque(sim.getCurrentDrawAmps()), sim.getVelocityMetersPerSecond() * positionToRotations * 2 * Math.PI));
    // sim.setInput(getVoltage());

    // Use motor voltage for TalonFX simulation input
    sim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    sim.update(0.020);

    // Convert meters to motor rotations
    double motorPosition =
      sim.getPositionMeters() * METERS_PER_ROTATION;
    double motorVelocity =
      sim.getVelocityMetersPerSecond() * METERS_PER_ROTATION;

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
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
    return getAngularPosition() * METERS_PER_ROTATION;
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
