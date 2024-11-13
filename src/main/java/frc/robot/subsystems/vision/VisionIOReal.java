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
            frontCamToRobot);
    }

    public void updateInputs(VisionIOInputs inputs) {
        updateEstimatedPose();
        updateVisibleToteAprilTags();
        inputs.estimatedRobotPose = estimatedRobotPose;
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
        visibleToteAprilTags = new Transform3d[12];
        for(PhotonTrackedTarget target : frontCam.getLatestResult().getTargets()) {
            if (target.getFiducialId() < 12) {
                visibleToteAprilTags[target.getFiducialId()] = target.getBestCameraToTarget();
            }
        }
     }
}
