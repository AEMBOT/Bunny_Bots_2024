package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot(PivotIO io) {
        this.io = io;
    }

    public void periodic() {}
}
