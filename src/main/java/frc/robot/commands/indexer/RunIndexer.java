package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerFlywheel;
import frc.robot.subsystems.indexer.IndexerSpindexer;
import frc.robot.subsystems.indexer.IndexerVerticalIndexer;

/**
 * Run specified parts of the indexer.
 */
public class RunIndexer extends Command {
    private final IndexerSpindexer spindexer;
    private final IndexerVerticalIndexer verticalIndexer;
    private final IndexerFlywheel flywheel;
    
    private final boolean runSpindexer;
    private final boolean runVerticalIndexer;
    private final boolean runFlywheel;
    
    /**
     * Run specified parts of the indexer.
     */
    public RunIndexer(
            IndexerSpindexer spindexer,
            IndexerVerticalIndexer verticalIndexer,
            IndexerFlywheel flywheel,
            boolean runSpindexerFlag,
            boolean runVerticalIndexerFlag,
            boolean runFlywheelFlag) {
        this.spindexer = spindexer;
        this.verticalIndexer = verticalIndexer;
        this.flywheel = flywheel;
        this.runSpindexer = runSpindexerFlag;
        this.runVerticalIndexer = runVerticalIndexerFlag;
        this.runFlywheel = runFlywheelFlag;
        
        // Add requirements for the subsystems we're using
        if (runSpindexerFlag) addRequirements(spindexer);
        if (runVerticalIndexerFlag) addRequirements(verticalIndexer);
        if (runFlywheelFlag) addRequirements(flywheel);
    }
    
    /**
     * Run all parts of the indexer.
     */
    public RunIndexer(
            IndexerSpindexer spindexer,
            IndexerVerticalIndexer verticalIndexer,
            IndexerFlywheel flywheel) {
        this(spindexer, verticalIndexer, flywheel, true, true, true);
    }
    
    @Override
    public void initialize() {
        if (runSpindexer) spindexer.run();
        if (runVerticalIndexer) verticalIndexer.run();
        if (runFlywheel) flywheel.spin();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (runSpindexer) spindexer.stop();
        if (runVerticalIndexer) verticalIndexer.stop();
        if (runFlywheel) flywheel.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
