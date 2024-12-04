package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    /**
     * Only looks at static april tags.
     * @return The Pose of the robot estimated by vision.
     */ 
    public Pose3d getVisionRobotPose() {
        return inputs.estimatedRobotPose;
    }

    /** 
     * @return The transform of the tote tag that is closest to the bot relative to the center of the bot. 
     */
    public Transform3d getTransformToClostestToteTag() {
        // Arbitrarily large transform so that all values are closer 
        Transform3d closestTagTransform = new Transform3d(1000,1000,1000, new Rotation3d());
        // For every visible tote april tag...
        for (Transform3d tagTransform : inputs.visibleToteAprilTags) {
            // If the distance from this tag to the bot 
            // is less than the distance to the current closest tag...
            if (new Translation3d().getDistance(
                tagTransform.getTranslation()) <
                 new Translation3d().getDistance(
                    tagTransform.getTranslation())) {
                // Set the closest tag to this one
                closestTagTransform = tagTransform;
            }        
        }
        return closestTagTransform;
    }
}
