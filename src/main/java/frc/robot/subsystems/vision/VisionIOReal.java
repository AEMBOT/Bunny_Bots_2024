package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOReal implements VisionIO {
    
    private final PhotonCamera frontCam;
    private final PhotonPoseEstimator frontCamPoseEstimator;

    private Optional<Pose3d> estimatedRobotPose = Optional.of(new Pose3d());

    public VisionIOReal() {
        // Front Cam
        frontCam = new PhotonCamera(frontCamName);
        frontCamPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCam,
            frontCamToRobot);
    }

    public void updateInputs(VisionIOInputs inputs) {
        updateEstimatedPose();
        inputs.estimatedRobotPose = estimatedRobotPose;
    }

    // Updates the pose of the robot estimated by vision
    public void updateEstimatedPose() {
        //Checks if the pose estimator has a value
        frontCamPoseEstimator.update().ifPresentOrElse(
            // If it does, set estimated robot pose to the estimated pose
            (value) -> { 
                estimatedRobotPose = Optional.of(value.estimatedPose); 
            },
            // If it doesnt, set estimated robot pose to an empty value
            () -> { 
                estimatedRobotPose = Optional.empty();
            }
         );
     }
}
