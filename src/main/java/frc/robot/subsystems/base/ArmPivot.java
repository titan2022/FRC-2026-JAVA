package frc.robot.subsystems.base;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import static frc.robot.ToSI.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Example of an arm or a pivot
 */
public class ArmPivot extends PositionPIDFBase {
  // If it's a pivot, use the example values below.
  // If it's an arm, get values from the CAD.
  public boolean IS_ARM = false;
  public double ARM_LENGTH = 0.1 * m;
  public double ARM_MOI = 0.01 * kg*m*m;
  // You can estimate it using SingleJointedArmSim.estimateMOI(armLength, 5).

  // Simulation
  protected final SingleJointedArmSim sim;

  protected final ArmPivotSimVisualization simVisualization;

  public ArmPivot() {
    super();

    if(RobotBase.isSimulation()) {
      sim = new SingleJointedArmSim(
        gearbox, // Motor type
        GEAR_RATIO,
        ARM_MOI, // Arm moment of inertia - Small value since there are no arm parameters
        ARM_LENGTH, // Arm length (m) - Small value since there are no arm parameters
        MIN_POSITION, // Min angle (rad)
        MAX_POSITION, // Max angle (rad)
        false, // Simulate gravity - Disable gravity for pivot
        STARTING_POSITION // Starting position (rad)
      );
      simVisualization = new ArmPivotSimVisualization(this);
    } else {
      sim = null;
      simVisualization = null;
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

    double motorPosition = Radians.of(sim.getAngleRads() * GEAR_RATIO).in(
      Rotations
    );
    double motorVelocity = RadiansPerSecond.of(
      sim.getVelocityRadPerSec() * GEAR_RATIO
    ).in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return sim;
  }

  /**
   * Get the simulation visualization.
   * @return The simulation visualization
   */
  public ArmPivotSimVisualization getSimVisualization() {
    return simVisualization;
  }
}
