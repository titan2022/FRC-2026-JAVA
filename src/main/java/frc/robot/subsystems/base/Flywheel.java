package frc.robot.subsystems.base;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import static frc.robot.ToSI.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Example of an arm or a pivot
 */
public class Flywheel extends VelocityPIDFBase {
  public double FLYWHEEL_MOI = 0.01 * kg*m*m;

  // Simulation
  protected FlywheelSim sim;

  // TODO - How to visualize a flywheel?
  // protected ArmPivotSimVisualization simVisualization;

  public Flywheel() {
  }

  // This method MUST be called at the end of subsystem initializers!
  @Override
  protected void initialize() {
    super.initialize();

    if(RobotBase.isSimulation()) {
      sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
          gearbox,
          FLYWHEEL_MOI,
          GEAR_RATIO),
        gearbox
      );
      // simVisualization = new ArmPivotSimVisualization(this);
    }
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
    sim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    sim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        sim.getCurrentDrawAmps()
      )
    );

    double motorVelocity = sim.getAngularVelocity().in(RotationsPerSecond);

    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public FlywheelSim getSimulation() {
    return sim;
  }

  // /**
  //  * Get the simulation visualization.
  //  * @return The simulation visualization
  //  */
  // public ArmPivotSimVisualization getSimVisualization() {
  //   return simVisualization;
  // }
}
