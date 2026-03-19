package frc.robot.localization;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
	public static class CameraInfo {
		public String cameraName;
		public Transform3d robotToCam;

		public CameraInfo(String cameraName, Transform3d robotToCam) {
			this.cameraName = cameraName;
			this.robotToCam = robotToCam;
		}
	}

	// back left side
	public static final CameraInfo camera10info = new CameraInfo(
		"10",
		new Transform3d(new Translation3d(-0.1375, 0.33, 0.05), new Rotation3d(Degrees.of(0), Degrees.of(65), Degrees.of(175.5)))
	);

	// back right side
	public static final CameraInfo camera11info = new CameraInfo(
		"11",
		new Transform3d(new Translation3d(-0.1375, -0.33, 0.05), new Rotation3d(Degrees.of(0), Degrees.of(65), Degrees.of(184.5)))
	);

	public static final CameraInfo[] cameraInfos = {
		camera10info,
		camera11info
	};

	// The standard deviations of our vision estimated poses, which affect correction rate
	// x, y, theta (metres, metres, radians)

	// recommended values by wither (FRC#4272 mentor)
	// https://discord.com/channels/176186766946992128/368993897495527424/1482932987115864106
	public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, Double.POSITIVE_INFINITY);
	public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.4, 0.4, Double.POSITIVE_INFINITY);

	public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	// Get the two tag poses for whichever alliance you're on
	private static final Pose3d tag10 = kTagLayout.getTagPose(10).get();
	private static final Pose3d tag4  = kTagLayout.getTagPose(4).get();

	// Midpoint translation = average of the two translations
	public static final Translation2d RED_HUB = new Translation2d(
			(tag10.getX() + tag4.getX()) / 2.0,
			(tag10.getY() + tag4.getY()) / 2.0
	);

	// Similarly for blue:
	private static final Pose3d tag20 = kTagLayout.getTagPose(20).get();
	private static final Pose3d tag26 = kTagLayout.getTagPose(26).get();

	public static final Translation2d BLUE_HUB = new Translation2d(
			(tag20.getX() + tag26.getX()) / 2.0,
			(tag20.getY() + tag26.getY()) / 2.0
	);

	public static void setupConstants() {
		// try {
		// 	kTagLayout = new AprilTagkTagLayout(Filesystem.getDeployDirectory().toPath().resolve("2026-rebuilt-welded.json"));
		// } catch(Throwable t) {
		// 	DriverStation.reportError("Failed to load 2026-rebuilt-welded.json", false);
		// }
	}
}
