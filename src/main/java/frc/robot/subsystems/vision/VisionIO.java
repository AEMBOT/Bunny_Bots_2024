package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {}

    // Updates the set of loggable inputs.
    public default void updateInputs(VisionIOInputs inputs) {}

    public default void getEstimatedPose() {}
}
