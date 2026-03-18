package frc.robot.subsystems.shooter;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class HubStatus {
  private HubStatus() {}

  public static boolean isOwnHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return false;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    // before game data arrives, assume active
    if (gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    // i think this is right but might have misread docs

    if (matchTime > 130.0) {
      return true;          // transition shift
    } else if (matchTime > 105.0) {
      return shift1Active;  // shift 1
    } else if (matchTime > 80.0) {
      return !shift1Active; // shift 2
    } else if (matchTime > 55.0) {
      return shift1Active;  // shift 3
    } else if (matchTime > 30.0) {
      return !shift1Active; // shift 4
    } else {
      return true;          // endgame
    }
  }
}