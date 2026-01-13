package frc.robot.drive;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.drive.pvswerve.SimSwerveDrivetrain;

public class DriveUtility {
  public static Drivetrain makeDrivetrain(Consumer<Pose2d> resetPose) {
    if(RobotBase.isReal()) {
      return new CTRESwerveDrivetrain();
    } else {
      return new SimSwerveDrivetrain(resetPose);
    }
  }
}
