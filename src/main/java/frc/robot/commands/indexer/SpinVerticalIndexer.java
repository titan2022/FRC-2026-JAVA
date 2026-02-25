package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerVerticalIndexer;

/**
 * Spin the vertical indexer (kicker + vertical tunnel rollers).
 */
public class SpinVerticalIndexer extends Command {
    private final IndexerVerticalIndexer verticalIndexer;
    
    public SpinVerticalIndexer(IndexerVerticalIndexer verticalIndexer) {
        this.verticalIndexer = verticalIndexer;
        
        addRequirements(verticalIndexer);
    }
    
    @Override
    public void initialize() {
        verticalIndexer.run();
    }
    
    @Override
    public void end(boolean interrupted) {
        verticalIndexer.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
