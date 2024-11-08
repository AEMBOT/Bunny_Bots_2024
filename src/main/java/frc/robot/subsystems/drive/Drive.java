package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drive extends SubsystemBase {
    
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    public Drive(DriveIO io) {
        this.io = io;
    }

    public void periodic() {}
}
