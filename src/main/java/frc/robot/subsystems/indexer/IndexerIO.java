package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {}

    // Updates the set of loggable inputs.
    public default void updateInputs(IndexerIOInputs inputs) {}

}
