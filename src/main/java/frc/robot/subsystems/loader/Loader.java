package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {
    
    private final LoaderIO io;
    private final LoaderIOInputsAutoLogged inputs = new LoaderIOInputsAutoLogged();

    public Loader(LoaderIO io) {
        this.io = io;
    }

    public void periodic() {}
}
