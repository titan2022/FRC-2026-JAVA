package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSpindexer;

/**
 * Spin the spindexer to feed balls into the vertical tunnel.
 */
public class SpinSpindexer extends Command {
    private final IndexerSpindexer spindexer;
    
    public SpinSpindexer(IndexerSpindexer spindexer) {
        this.spindexer = spindexer;
        
        addRequirements(spindexer);
    }
    
    @Override
    public void initialize() {
        spindexer.run();
    }
    
    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
