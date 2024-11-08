package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {}

    // Updates the set of loggable inputs.
    public default void updateInputs(PivotIOInputs inputs) {}

}
