package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Constants.HardwareConstants;
import frc.robot.drive.ctre.CTRESwerveDrivetrain;
import frc.robot.subsystems.base.VoltageControlledBase;

public class Intake extends VoltageControlledBase {
  private IntakeSimulation intakeSimulation = null;

  // TODO: Set to actual voltage
  private static final double INTAKE_VOLTAGE = 1.0;

  {
    SUBSYSTEM_NAME = "Intake";

    // Hardware
    motor = new TalonFX(42, HardwareConstants.rioCanbus);

    // Base defaults
    DEFAULT_VOLTAGE = INTAKE_VOLTAGE;

    // Motor config
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // TODO - Figure out what supply and stator current limits we want
    // motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    // motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    // motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    // motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

  }

  public Intake(CTRESwerveDrivetrain drivetrain) {
    if(RobotBase.isSimulation()) {
      // AbstractDriveTrainSimulation driveTrainSimulation = (drivetrain.mapleSimSwerveDrivetrain != null) 
      //   ? drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive
      //   : null;
      // this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
      //   // Specify the type of game pieces that the intake can collect
      //   "Fuel",
      //   // Specify the drivetrain to which this intake is attached
      //   driveTrainSimulation,
      //   // Width of the intake
      //   Inches.of(25.375),
      //   // The extension length of the intake beyond the robot's frame (when activated)
      //   Inches.of(8.360),
      //   // The intake is mounted on the front side of the chassis
      //   IntakeSimulation.IntakeSide.FRONT,
      //   // The intake can hold up to 100 notes
      //   100);
    }
    initialize();
  }

  public Command intakeCommand() {
    return setVoltageCommand(DEFAULT_VOLTAGE)
      .alongWith(new InstantCommand(() -> setIsOut(true)));
  }

  @Override
  public void stop() {
    super.stop();
    setIsOut(false);
  }

  /// Sets whether or not the intake is out, for the purpose of simulation
  public void setIsOut(boolean isOut) {
    if(RobotBase.isSimulation()) {
      if (isOut)
        intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
      else
        intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }
  }

  /// Returns the number of balls in the hopper. When on the real bot, returns 0.
  public int simGetNumberOfBallsInHopper() {
    if(RobotBase.isSimulation()) {
      return intakeSimulation.getGamePiecesAmount();
    } else {
      return 0;
    }
  }

  /// If there are any game pieces remaining in the simulated hopper, retrieves it
  /// from the hopper. If there are no game pieces remaining, returns false.
  /// On the real bot, returns true.
  public boolean simRetrieveBallFromHopper() {
    if(RobotBase.isSimulation()) {
      return intakeSimulation.obtainGamePieceFromIntake();
    } else {
      return true;
    }
  }

  public Command simRetrieveBallFromHopperCommand() {
    return new InstantCommand(this::simRetrieveBallFromHopper);
  }

  @Override
  public void periodic() {
    super.periodic();

    // DogLog.log(SUBSYSTEM_NAME + "/Number of balls in hopper", simGetNumberOfBallsInHopper());
  }

  private class SimPeriodicallyRetrieveBallFromHopperCommand extends Command {
    private long counter;
    private final long maxCounterValue;
    private final Intake intake;

    /// Retrieves a ball from the simulated hopper every `period` milliseconds.
    public SimPeriodicallyRetrieveBallFromHopperCommand(Intake intake, long period) {
      this.maxCounterValue = period;
      this.intake = intake;
      this.counter = 0;
    }

    @Override
    public void execute() {
      counter += 20;
      if(counter >= maxCounterValue) {
        counter = 0;
        intake.simRetrieveBallFromHopper();
      }
    } 
  }

  /// Retrieves a ball from the simulated hopper every `period` milliseconds.
  public Command simPeriodicallyRetrieveBallFromHopperCommand(long period) {
    return new SimPeriodicallyRetrieveBallFromHopperCommand(this, period);
  }

}

