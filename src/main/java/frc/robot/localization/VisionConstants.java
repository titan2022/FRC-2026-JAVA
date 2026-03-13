package frc.robot.localization;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

	public static final CameraInfo camera10info = new CameraInfo(
		"10",
		new Transform3d(new Translation3d(-0.1375, 0.33, 0.05), new Rotation3d(Degrees.of(0), Degrees.of(65), Degrees.of(175.5)))
	);

	public static final CameraInfo camera11info = new CameraInfo(
		"11",
		new Transform3d(new Translation3d(-0.1375, -0.33, 0.05), new Rotation3d(Degrees.of(0), Degrees.of(65), Degrees.of(184.5)))
	);

	public static final CameraInfo[] cameraInfos = {
		camera10info,
		camera11info
	};

	// The standard deviations of our vision estimated poses, which affect correction rate
	// (Fake values. Experiment and determine estimation noise on an actual robot.)
	public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
	public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

	public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public static void setupConstants() {
		// try {
		// 	kTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2026-rebuilt-welded.json"));
		// } catch(Throwable t) {
		// 	DriverStation.reportError("Failed to load 2026-rebuilt-welded.json", false);
		// }
	}
}
