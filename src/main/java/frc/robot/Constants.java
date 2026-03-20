// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.ToSI.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriverConstants {
    /// The max speed, in meters per second
    public static final double MAX_SPEED = 6.0 * m/s;
    // alternately TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /// The max angular speed, in radians per second
    public static final double MAX_ANGULAR_SPEED = DegreesPerSecond.of(400).in(RadiansPerSecond);
    /// The deadband, as a fraction of 1
    public static final double DEADBAND = 0.15;
    /// The speed of dpad strafing, in meters per second
    public static final double DPAD_STRAFE_SPEED = 0.5 * m/s;
  }

  public static class HardwareConstants {
    public static final CANBus rioCanbus = new CANBus();
    public static final CANBus canivoreCanbus = new CANBus("FRC2022-2");
  }

  public static class ShootingConstants {
    public static final double shootToAllianceArea_firingPitch = 10.0;
    public static final double shootToAllianceArea_shooterYaw = 10.0;

    public static final Angle yaw = Rotations.of(-0.25);
  }
}
