package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOReal implements VisionIO {
    
    private final PhotonCamera frontCam;
    private final PhotonPoseEstimator frontCamPoseEstimator;

    private Pose3d estimatedRobotPose = new Pose3d();
    public Transform3d[] visibleToteAprilTags = new Transform3d[12];

    public VisionIOReal() {
        // Front Cam
        frontCam = new PhotonCamera(frontCamName);
        frontCamPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCam,
            frontCamFromRobot);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        updateEstimatedPose();
        updateVisibleToteAprilTags();
        inputs.estimatedRobotPose = estimatedRobotPose;
        inputs.visibleToteAprilTags = visibleToteAprilTags;
    }

    // Updates the pose of the robot estimated by vision
    public void updateEstimatedPose() {
        //Checks if the pose estimator has a value
        frontCamPoseEstimator.update().ifPresentOrElse(
            // If it does, set estimated robot pose to the estimated pose
            (value) -> { 
                estimatedRobotPose = value.estimatedPose; 
            },
            // If it doesnt, set estimated robot pose to an empty value
            () -> { 
                estimatedRobotPose = null;
            }
         );
     }

     public void updateVisibleToteAprilTags() {
        // Reset the list of tote april tags to an empty one
        visibleToteAprilTags = new Transform3d[12];
        // For every visible target...
        for(PhotonTrackedTarget target : frontCam.getLatestResult().getTargets()) {
            // If its ID is less than 12...
            if (target.getFiducialId() < 12) {
                // Set the index of its ID to the transform from the camera to the target
                visibleToteAprilTags[target.getFiducialId()] = target.getBestCameraToTarget().plus(frontCamFromRobot);
            }
        }
     }
}
