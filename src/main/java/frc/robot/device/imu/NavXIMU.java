package frc.robot.device.imu;

import com.studica.frc.AHRS;

import edu.wpi.first.units.measure.Angle;

public class NavXIMU extends AHRS implements IMU {
  public NavXIMU(AHRS.NavXComType comType) {
    super(comType);
  }

  public NavXIMU(AHRS.NavXComType comType, int customRateHz) {
    super(comType, customRateHz);
  }

  public NavXIMU(AHRS.NavXComType comType, AHRS.NavXUpdateRate updateRate) {
    super(comType, updateRate);
  }
  
  public void simSetRawYaw(Angle yaw) {
    // no-op, there's no simulator
  }

  public boolean isNavX() {
    return true;
  }
}
