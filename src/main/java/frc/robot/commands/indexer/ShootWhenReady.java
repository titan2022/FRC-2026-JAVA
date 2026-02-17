package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerFlywheel;
import frc.robot.subsystems.indexer.IndexerSpindexer;
import frc.robot.subsystems.indexer.IndexerVerticalIndexer;

/**
 * Spins up flywheel, then runs indexers once flywheel is at speed.
 * This ensures balls are only fed when the flywheel is ready to shoot.
 */
public class ShootWhenReady extends Command {
    private final IndexerSpindexer spindexer;
    private final IndexerVerticalIndexer verticalIndexer;
    private final IndexerFlywheel flywheel;
    
    private boolean flywheelReady = false;
    
    public ShootWhenReady(
            IndexerSpindexer spindexer,
            IndexerVerticalIndexer verticalIndexer,
            IndexerFlywheel flywheel) {
        this.spindexer = spindexer;
        this.verticalIndexer = verticalIndexer;
        this.flywheel = flywheel;
        
        addRequirements(spindexer, verticalIndexer, flywheel);
    }
    
    @Override
    public void initialize() {
        flywheelReady = false;
        flywheel.spin();
    }
    
    @Override
    public void execute() {
        if (!flywheelReady && flywheel.isAtSpeed()) {
            flywheelReady = true;
            spindexer.run();
            verticalIndexer.run();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
        verticalIndexer.stop();
        flywheel.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
