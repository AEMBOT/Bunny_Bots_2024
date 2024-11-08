package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.AutoLog;

public interface LoaderIO {
    
    @AutoLog
    public static class LoaderIOInputs {}

    // Updates the set of loggable inputs.
    public default void updateInputs(LoaderIOInputs inputs) {}

}
