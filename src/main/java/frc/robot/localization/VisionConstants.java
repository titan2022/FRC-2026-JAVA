package frc.robot.localization;

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

	public static final CameraInfo camera1info = new CameraInfo(
		"1",
		new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
	);

	public static final CameraInfo camera2info = new CameraInfo(
		"2",
		new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, Math.PI))
	);

	public static final CameraInfo[] cameraInfos = {
		camera1info,
		camera2info
	};

	// The standard deviations of our vision estimated poses, which affect correction rate
	// (Fake values. Experiment and determine estimation noise on an actual robot.)
	public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
	public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

	public static AprilTagFieldLayout kTagLayout = null;

	public static void setupConstants() {
		try {
			kTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2026-rebuilt-welded.json"));
		} catch(Throwable t) {
			DriverStation.reportError("Failed to load 2026-rebuilt-welded.json", false);
		}
	}
}
