package frc.robot.device.imu;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;

public class PigeonIMU extends Pigeon2 implements IMU {
  public PigeonIMU(int deviceId, String canbus) {
    super(deviceId, canbus);
  }

  public PigeonIMU(int deviceId, CANBus canbus) {
    super(deviceId, canbus);
  }

  public PigeonIMU(int deviceId) {
    super(deviceId);
  }
  
  public void simSetRawYaw(Angle yaw) {
    getSimState().setRawYaw(yaw);
  }

  public boolean isPigeon() {
    return true;
  }
}
