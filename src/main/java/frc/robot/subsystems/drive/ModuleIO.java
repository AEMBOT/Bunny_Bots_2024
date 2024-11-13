package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {}

    // Updates the set of loggable inputs.
    public default void updateInputs(ModuleIOInputs inputs) {}

}
