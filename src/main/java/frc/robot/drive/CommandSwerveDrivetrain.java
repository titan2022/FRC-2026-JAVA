package frc.robot.drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface CommandSwerveDrivetrain extends Subsystem {
  /**
	 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
	 * while still accounting for measurement noise.
	 *
	 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds The timestamp of the vision measurement in seconds.
	 */
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
	 * while still accounting for measurement noise.
	 * <p>
	 * Note that the vision measurement standard deviations passed into this method
	 * will continue to apply to future measurements until a subsequent call to
	 * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
	 *
	 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds The timestamp of the vision measurement in seconds.
	 * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
	 *     in the form [x, y, theta]ᵀ, with units in meters and radians.
	 */
	public void addVisionMeasurement(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs
	);

  public void resetFieldOrientation();

	public void setVelocities(ChassisSpeeds speeds);

	public void setFieldVelocities(ChassisSpeeds speeds);

	public ChassisSpeeds getVelocities();

	public void brake();
}
