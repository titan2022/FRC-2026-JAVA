package frc.robot.subsystems.base;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Visualization for the arm subsystem in simulation.
 */
public class ArmPivotSimVisualization extends SubsystemBase {
  private final ArmPivot arm;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d armMech;

  // Visualization constants
  private final double BASE_WIDTH = 40.0;
  private final double BASE_HEIGHT = 20.0;
  private final double TOWER_HEIGHT = 30.0;
  private final double ARM_WIDTH = 10.0;

  // Arm parameters
  private final double armLength;
  private final double visualScaleFactor;

  /**
   * Creates a new visualization for the arm.
   *
   * @param armSubsystem The arm subsystem to visualize
   */
  public ArmPivotSimVisualization(ArmPivot armSubsystem) {
    this.arm = armSubsystem;
    this.armLength = armSubsystem.ARM_LENGTH;

    // Calculate scale factor to keep visualization in reasonable bounds
    visualScaleFactor = 200.0 / armLength; // Scale to ~200 pixels

    // Create the simulation display
    mech = new Mechanism2d(400, 400);
    root = mech.getRoot("ArmRoot", 200, 200);

    // Add arm base
    MechanismLigament2d armBase = root.append(
      new MechanismLigament2d(
        "Base",
        BASE_WIDTH,
        0,
        BASE_HEIGHT,
        new Color8Bit(Color.kDarkGray)
      )
    );

    // Add tower
    MechanismLigament2d tower = armBase.append(
      new MechanismLigament2d(
        "Tower",
        TOWER_HEIGHT,
        90,
        BASE_HEIGHT / 2,
        new Color8Bit(Color.kGray)
      )
    );

    // Add the arm pivot point
    MechanismLigament2d pivot = tower.append(
      new MechanismLigament2d("Pivot", 5, 0, 5, new Color8Bit(Color.kBlack))
    );

    // Add the arm
    armMech = pivot.append(
      new MechanismLigament2d(
        "Arm",
        armLength * visualScaleFactor,
        0,
        ARM_WIDTH,
        new Color8Bit(Color.kBlue)
      )
    );

    // Initialize visualization
    SmartDashboard.putData(arm.SUBSYSTEM_NAME, mech);
  }

  @Override
  public void periodic() {
    // Update arm angle
    double currentAngleRad = arm.getSimulation().getAngleRads();
    armMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    // Add telemetry data
    SmartDashboard.putNumber(
      arm.SUBSYSTEM_NAME + "/Simulation/Angle (deg)",
      Units.radiansToDegrees(currentAngleRad)
    );
    SmartDashboard.putNumber(
      arm.SUBSYSTEM_NAME + "/Simulation/Velocity (deg/s)",
      Units.radiansToDegrees(arm.getSimulation().getVelocityRadPerSec())
    );
    SmartDashboard.putNumber(
      arm.SUBSYSTEM_NAME + "/Simulation/Current (A)",
      arm.getSimulation().getCurrentDrawAmps()
    );
  }
}
