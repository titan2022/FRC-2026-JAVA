package frc.robot.drive.ctre.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.SwerveDrivetrain;

public class DriveToPose extends SequentialCommandGroup {
    public DriveToPose(
        SwerveDrivetrain drivetrain,
        Pose2d targetPose,
        PathConstraints constraints
    ) {
        addCommands(
            AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0.0),
            Commands.runOnce(drivetrain::brake, drivetrain)
        );
    }
}