package frc.robot.drive.kitbot;

public class KitbotDriveConstants {
	// Motor controller IDs for drivetrain motors
	public static final int LEFT_LEADER_ID = 1;
	public static final int LEFT_FOLLOWER_ID = 2;
	public static final int RIGHT_LEADER_ID = 3;
	public static final int RIGHT_FOLLOWER_ID = 4;

	// Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
	// likelihood of tripping breakers or damaging CIM motors
	public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
}
