package frc.robot.device.imu;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

public interface IMU {
  public Rotation2d getRotation2d();

  default public void simSetRawYaw(Angle yaw) {
    simSetRawYaw(new Rotation2d(yaw));
  }

  default public void simSetRawYaw(Rotation2d yaw) {
    simSetRawYaw(Radians.of(yaw.getRadians()));
  }

  default public boolean isPigeon() {
    return false;
  }

  default public boolean isNavX() {
    return false;
  }

  public static IMU getKitbotIMU(int pigeonCanID) {
    if(RobotBase.isReal()) {
      // https://pdocs.kauailabs.com/navx-mxp/guidance/selecting-an-interface/
      // – If mounting the navX-sensor directly on the RoboRIO, the SPI interface is preferred for it’s high speed and low latency.
      return new NavXIMU(NavXIMU.NavXComType.kMXP_SPI);
    } else {
      return new PigeonIMU(pigeonCanID);
    }
  }
}
