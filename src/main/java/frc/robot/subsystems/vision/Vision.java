package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public Pose3d getVisionRobotPose() {
     if (inputs.estimatedRobotPose.isPresent()) {
        return inputs.estimatedRobotPose.get();
     }
    return null;
    }
}
