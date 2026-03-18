package frc.robot.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class StaticShotCalculator {
  public record StaticShotSolution(
      boolean valid,
      Rotation2d driveHeading,
      double distanceMeters,
      double flywheelRps,
      double pitchDegrees) {}

  // move constants?
  // field / hub geometry
  private static final double FIELD_LENGTH_METERS = 16.54; // 651.2 in
  private static final double FIELD_WIDTH_METERS = 8.07;   // 317.7 in
  private static final double HUB_X_FROM_ALLIANCE_WALL_METERS = 4.03; // 158.6 in
  private static final double HUB_CENTER_Y_METERS = FIELD_WIDTH_METERS / 2.0;

  // shooting zone for the lookup table below
  private static final double MIN_DISTANCE_METERS = 1.50;
  private static final double MAX_DISTANCE_METERS = 5.50;

  private final InterpolatingDoubleTreeMap pitchDegreesMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelRpsMap = new InterpolatingDoubleTreeMap();

  public StaticShotCalculator() {
    // replace with actual data
    // distance (m), pitch subsystem degrees, flywheel rps
    addShot(1.50, 102.0, 45.0);
    addShot(2.00, 106.0, 48.0);
    addShot(2.50, 110.0, 52.0);
    addShot(3.00, 114.0, 56.0);
    addShot(3.50, 118.0, 60.0);
    addShot(4.00, 122.0, 65.0);
    addShot(4.50, 126.0, 70.0);
    addShot(5.00, 130.0, 75.0);
    addShot(5.50, 134.0, 80.0);
  }

  public void addShot(double distanceMeters, double pitchDegrees, double flywheelRps) {
    pitchDegreesMap.put(distanceMeters, pitchDegrees);
    flywheelRpsMap.put(distanceMeters, flywheelRps);
  }

  public Translation2d getAllianceHubCenter() {
    // not sure if this is right way to do it
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (alliance == Alliance.Blue) {
      return new Translation2d(HUB_X_FROM_ALLIANCE_WALL_METERS, HUB_CENTER_Y_METERS);
    }

    return new Translation2d(
        FIELD_LENGTH_METERS - HUB_X_FROM_ALLIANCE_WALL_METERS,
        HUB_CENTER_Y_METERS);
  }

  public StaticShotSolution calculate(Pose2d robotPose) {
    Translation2d target = getAllianceHubCenter();
    Translation2d toTarget = target.minus(robotPose.getTranslation());

    double distanceMeters = toTarget.getNorm();
    Rotation2d driveHeading = new Rotation2d(toTarget.getX(), toTarget.getY());

    double clampedDistance = Math.max(MIN_DISTANCE_METERS, Math.min(MAX_DISTANCE_METERS, distanceMeters));
    boolean valid = distanceMeters >= MIN_DISTANCE_METERS && distanceMeters <= MAX_DISTANCE_METERS;

    double pitchDegrees = pitchDegreesMap.get(clampedDistance);
    double flywheelRps = flywheelRpsMap.get(clampedDistance);

    DogLog.log("Shooter/StaticShot/DistanceMeters", distanceMeters, "m");
    DogLog.log("Shooter/StaticShot/DriveHeadingDegrees", driveHeading.getDegrees(), "deg");
    DogLog.log("Shooter/StaticShot/PitchDegrees", pitchDegrees, "deg");
    DogLog.log("Shooter/StaticShot/FlywheelRps", flywheelRps, "rps");
    DogLog.log("Shooter/StaticShot/Valid", valid);

    return new StaticShotSolution(valid, driveHeading, distanceMeters, flywheelRps, pitchDegrees);
  }
}