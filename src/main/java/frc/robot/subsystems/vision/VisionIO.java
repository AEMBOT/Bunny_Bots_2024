package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        public Pose3d estimatedRobotPose = new Pose3d();
    }

    // Updates the set of loggable inputs.
    public default void updateInputs(VisionIOInputs inputs) {}

    // Updates the refrence pose of vision simulation
    public default void updatePose(Pose2d pose) {}

    // Gets the estimated pose from vision
    public default void updateEstimatedPose() {}

}
