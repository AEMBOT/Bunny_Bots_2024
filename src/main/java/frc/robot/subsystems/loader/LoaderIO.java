package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.AutoLog;

public interface LoaderIO {
    @AutoLog
    public static class LoaderIOInputs {
        public double motorAppliedVolts = 0.0;

        public double motorCurrentAmps = 0.0;
    }

    // Updates the set of loggable inputs.
    public default void updateInputs(LoaderIOInputs inputs) {}

    public default void setMotorVolts(double voltage) {}
}
