package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {}

    // Updates the set of loggable inputs.
    public default void updateInputs(ShooterIOInputs inputs) {}

}
