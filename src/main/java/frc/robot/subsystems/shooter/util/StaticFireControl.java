package frc.robot.subsystems.shooter.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

// import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.localization.VisionConstants;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.shooter.util.FieldConstants.LinesVertical;

// https://github.com/FRC1257/2026-Robot/blob/Sam's-Branch/src/main/java/frc/robot/subsystems/Shooter/ShooterTrajectoryCalculator.java
public class StaticFireControl {
	public record ShooterTrajectoryParameters(
		boolean isValid, 
		Rotation2d driveAngle,
		double driveVelocity,
		AngularVelocity flywheelSpeed,
		Angle pitch,
		boolean passing
	) {}

	private static final InterpolatingTreeMap<Distance, Angle> pitchMap =
		new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(Radians));
	private static final InterpolatingTreeMap<Distance, AngularVelocity> flywheelSpeedMap =
		new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(RadiansPerSecond));

	private static final Distance MIN_SHOOTING_DISTANCE = Meters.of(1.9);
	private static final Distance MAX_SHOOTING_DISTANCE = Meters.of(5.2);

	private static final Distance MIN_PASSING_DISTANCE = Meters.of(1.9);
	private static final Distance MAX_PASSING_DISTANCE = Meters.of(5.2);

	static {
		pitchMap.put(Meters.of(0.96 + 0.5969 + 0.3429), Radians.of(0.0));
		pitchMap.put(Meters.of(1.16 + 0.5969 + 0.3429), Radians.of(0.025));
		pitchMap.put(Meters.of(1.58 + 0.5969 + 0.3429), Radians.of(0.032));
		pitchMap.put(Meters.of(2.07 + 0.5969 + 0.3429), Radians.of(0.039));
		pitchMap.put(Meters.of(2.37 + 0.5969 + 0.3429), Radians.of(0.048));
		pitchMap.put(Meters.of(2.47 + 0.5969 + 0.3429), Radians.of(0.053));
		pitchMap.put(Meters.of(2.70 + 0.5969 + 0.3429), Radians.of(0.061));
		pitchMap.put(Meters.of(2.94 + 0.5969 + 0.3429), Radians.of(0.065));
		pitchMap.put(Meters.of(3.48 + 0.5969 + 0.3429), Radians.of(0.065));
		pitchMap.put(Meters.of(3.92 + 0.5969 + 0.3429), Radians.of(0.15));
		pitchMap.put(Meters.of(4.35 + 0.5969 + 0.3429), Radians.of(0.158));

		flywheelSpeedMap.put(Meters.of(0.96 + 0.5969 + 0.3429), RadiansPerSecond.of(300));
		flywheelSpeedMap.put(Meters.of(1.16 + 0.5969 + 0.3429), RadiansPerSecond.of(300));
		flywheelSpeedMap.put(Meters.of(1.58 + 0.5969 + 0.3429), RadiansPerSecond.of(315));
		flywheelSpeedMap.put(Meters.of(2.07 + 0.5969 + 0.3429), RadiansPerSecond.of(330));
		flywheelSpeedMap.put(Meters.of(2.37 + 0.5969 + 0.3429), RadiansPerSecond.of(340));
		flywheelSpeedMap.put(Meters.of(2.47 + 0.5969 + 0.3429), RadiansPerSecond.of(350));
		flywheelSpeedMap.put(Meters.of(2.70 + 0.5969 + 0.3429), RadiansPerSecond.of(355));
		flywheelSpeedMap.put(Meters.of(2.94 + 0.5969 + 0.3429), RadiansPerSecond.of(360));
		flywheelSpeedMap.put(Meters.of(3.48 + 0.5969 + 0.3429), RadiansPerSecond.of(362));
		flywheelSpeedMap.put(Meters.of(3.92 + 0.5969 + 0.3429), RadiansPerSecond.of(370));
		flywheelSpeedMap.put(Meters.of(4.35 + 0.5969 + 0.3429), RadiansPerSecond.of(390));
	}

	private static final InterpolatingTreeMap<Distance, Angle> passingPitchMap =
		new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(Radians));
	private static final InterpolatingTreeMap<Distance, AngularVelocity> passingFlywheelSpeedMap =
		new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(RadiansPerSecond));

	static {
		passingPitchMap.put(Meters.of(0.96 + 0.5969 + 0.3429), Radians.of(0.0));
		passingPitchMap.put(Meters.of(1.16 + 0.5969 + 0.3429), Radians.of(0.025));
		passingPitchMap.put(Meters.of(1.58 + 0.5969 + 0.3429), Radians.of(0.032));
		passingPitchMap.put(Meters.of(2.07 + 0.5969 + 0.3429), Radians.of(0.039));
		passingPitchMap.put(Meters.of(2.37 + 0.5969 + 0.3429), Radians.of(0.048));
		passingPitchMap.put(Meters.of(2.47 + 0.5969 + 0.3429), Radians.of(0.053));
		passingPitchMap.put(Meters.of(2.70 + 0.5969 + 0.3429), Radians.of(0.061));
		passingPitchMap.put(Meters.of(2.94 + 0.5969 + 0.3429), Radians.of(0.065));
		passingPitchMap.put(Meters.of(3.48 + 0.5969 + 0.3429), Radians.of(0.065));
		passingPitchMap.put(Meters.of(3.92 + 0.5969 + 0.3429), Radians.of(0.15));
		passingPitchMap.put(Meters.of(4.35 + 0.5969 + 0.3429), Radians.of(0.158));

		passingFlywheelSpeedMap.put(Meters.of(0.96 + 0.5969 + 0.3429), RadiansPerSecond.of(300));
		passingFlywheelSpeedMap.put(Meters.of(1.16 + 0.5969 + 0.3429), RadiansPerSecond.of(300));
		passingFlywheelSpeedMap.put(Meters.of(1.58 + 0.5969 + 0.3429), RadiansPerSecond.of(315));
		passingFlywheelSpeedMap.put(Meters.of(2.07 + 0.5969 + 0.3429), RadiansPerSecond.of(330));
		passingFlywheelSpeedMap.put(Meters.of(2.37 + 0.5969 + 0.3429), RadiansPerSecond.of(340));
		passingFlywheelSpeedMap.put(Meters.of(2.47 + 0.5969 + 0.3429), RadiansPerSecond.of(350));
		passingFlywheelSpeedMap.put(Meters.of(2.70 + 0.5969 + 0.3429), RadiansPerSecond.of(355));
		passingFlywheelSpeedMap.put(Meters.of(2.94 + 0.5969 + 0.3429), RadiansPerSecond.of(360));
		passingFlywheelSpeedMap.put(Meters.of(3.48 + 0.5969 + 0.3429), RadiansPerSecond.of(362));
		passingFlywheelSpeedMap.put(Meters.of(3.92 + 0.5969 + 0.3429), RadiansPerSecond.of(370));
		passingFlywheelSpeedMap.put(Meters.of(4.35 + 0.5969 + 0.3429), RadiansPerSecond.of(390));
	}

	// https://claude.ai/share/6b5f99cd-cdc6-46b8-b01c-a0e8c5c1e5bb
	public static Translation2d getPassTarget(Translation2d robotPosition) {
    double targetX = LinesVertical.allianceZone / 2.0; // centerline of infield
    double clampedY = Math.max(0.0, Math.min(FieldConstants.fieldWidth, robotPosition.getY()));
    return new Translation2d(targetX, clampedY);
	}

	// We make the assumption that the robot is travelling parallel to the DS wall.
	public static ShooterTrajectoryParameters getPassingParameters(Pose2d robotPose) {
		Translation2d robotField = AllianceFlipUtil.apply(robotPose.getTranslation());
		Translation2d target = AllianceFlipUtil.apply(getPassTarget(robotField));

		Distance robotToTargetDistance = Meters.of(target.getDistance(robotPose.getTranslation()));
		// DogLog.log("Shooter/TrajectoryCalculator/PassingDistance", robotToTargetDistance);

		boolean inRange = robotToTargetDistance.gte(MIN_PASSING_DISTANCE)
		                && robotToTargetDistance.lte(MAX_PASSING_DISTANCE);

    return new ShooterTrajectoryParameters(
			inRange,
			robotPose.getRotation(), // don't modify drive heading
			0,
			passingFlywheelSpeedMap.get(robotToTargetDistance),
			passingPitchMap.get(robotToTargetDistance),
			true
    );
	}

	public static ShooterTrajectoryParameters getStaticParameters(Pose2d robotPose) {
		Translation2d target = VisionConstants.getHubPosition();
		Distance robotToTargetDistance = Meters.of(target.getDistance(robotPose.getTranslation()));
		// DogLog.log("Shooter/TrajectoryCalculator/HubDistance", robotToTargetDistance);
		boolean inRange = robotToTargetDistance.gte(MIN_SHOOTING_DISTANCE) 
                   && robotToTargetDistance.lte(MAX_SHOOTING_DISTANCE);
		return new ShooterTrajectoryParameters(inRange, robotPose.getRotation(), 0, flywheelSpeedMap.get(robotToTargetDistance), pitchMap.get(robotToTargetDistance), false);
	}

	public static Command staticAutoShootCommand(CTRESwerveDrivetrain drivetrain, ShooterPitch shooterPitch, ShooterFlywheel shooterFlywheel) {
		return Commands.run(() -> {
			ShooterTrajectoryParameters parameters = getStaticParameters(drivetrain.getPose());
			shooterPitch.setAngularPosition(parameters.pitch.in(Rotations));
			shooterFlywheel.setAngularVelocity(parameters.flywheelSpeed.in(RotationsPerSecond));
		}, shooterPitch, shooterFlywheel);
	}

	// We make the assumption that the robot is travelling parallel to the DS wall.
	public static Command passingAutoShootCommand(CTRESwerveDrivetrain drivetrain, ShooterPitch shooterPitch, ShooterFlywheel shooterFlywheel) {
		return Commands.run(() -> {
			ShooterTrajectoryParameters parameters = getPassingParameters(drivetrain.getPose());
			shooterPitch.setAngularPosition(parameters.pitch.in(Rotations));
			shooterFlywheel.setAngularVelocity(parameters.flywheelSpeed.in(RotationsPerSecond));
		}, shooterPitch, shooterFlywheel);
	}
}

