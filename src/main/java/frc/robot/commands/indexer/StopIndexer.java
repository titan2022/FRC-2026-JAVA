package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.indexer.IndexerFlywheel;
import frc.robot.subsystems.indexer.IndexerSpindexer;
import frc.robot.subsystems.indexer.IndexerVerticalIndexer;

/**
 * Instantly stops all indexer components.
 */
public class StopIndexer extends InstantCommand {
    
    public StopIndexer(
            IndexerSpindexer spindexer,
            IndexerVerticalIndexer verticalIndexer,
            IndexerFlywheel flywheel) {
        super(
            () -> {
                spindexer.stop();
                verticalIndexer.stop();
                flywheel.stop();
            },
            spindexer, verticalIndexer, flywheel
        );
    }
}
