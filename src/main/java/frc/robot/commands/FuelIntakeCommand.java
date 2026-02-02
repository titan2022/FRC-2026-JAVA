package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIntakeSubsystem;

/**
 * Intakes fuel.
 */
public class FuelIntakeCommand extends Command {
  private final FuelIntakeSubsystem intake;

  public FuelIntakeCommand(FuelIntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.runIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}