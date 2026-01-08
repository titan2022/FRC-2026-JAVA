package frc.robot.drive;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.drive.sim.SimSwerveDrivetrain;

public class DriveUtility {
  public static CommandSwerveDrivetrain makeDrivetrain() {
    if(RobotBase.isReal()) {
      return new CTRESwerveDrivetrain();
    } else {
      return new SimSwerveDrivetrain();
    }
  }
}
