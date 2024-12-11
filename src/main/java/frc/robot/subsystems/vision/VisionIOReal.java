package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.stream.Stream;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionIOReal implements VisionIO {
    
    private final PhotonCamera frontCam;
    private final PhotonPoseEstimator frontCamPoseEstimator;

    private final PhotonCamera rightCam;
    private final PhotonPoseEstimator rightCamPoseEstimator;

    private final PhotonCamera leftCam;
    private final PhotonPoseEstimator leftCamPoseEstimator;

    private Pose3d estimatedRobotPose = new Pose3d();
    public Transform3d[] visibleToteAprilTags = new Transform3d[12];

    public VisionIOReal() {
        // Front camera and object that estimates robot pose from front cam input
        frontCam = new PhotonCamera(frontCamName);
        frontCamPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCam,
            frontCamFromRobot);

        // Right camera and object that estimates robot pose from right cam input
        rightCam = new PhotonCamera(frontCamName);
        rightCamPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            rightCam,
            rightCamFromRobot);

        // Left camera and object that estimates robot pose from left cam input
        leftCam = new PhotonCamera(frontCamName);
        leftCamPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            leftCam,
            leftCamFromRobot);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        
        if (aprilTagFieldLayout != null) {
            updateEstimatedPose();
            updateVisibleToteAprilTags();
        }

        inputs.estimatedRobotPose = estimatedRobotPose;
        inputs.visibleToteAprilTags = visibleToteAprilTags;
    }

    private Pose3d getUpdatedIndiviualCameraEstimatedPose(PhotonPoseEstimator poseEstimator) {
        // If the pose estimator has a value then return the pose
        try {
            return poseEstimator.update().get().estimatedPose;
        // Otherwise, return the current pose of the robot that is estimated by vision
        // In practice, this should have the effect of valuing estimated poses more the more camera see tags
        } catch(NoSuchElementException e) {
            return estimatedRobotPose;
        }

    }

    public void updateEstimatedPose() {

        // Initialize a temporary pose
        Pose3d averagePose = new Pose3d();

        // Add all three poses together
        averagePose = averagePose.plus(new Transform3d(new Pose3d(), getUpdatedIndiviualCameraEstimatedPose(frontCamPoseEstimator)));
        averagePose = averagePose.plus(new Transform3d(new Pose3d(), getUpdatedIndiviualCameraEstimatedPose(rightCamPoseEstimator)));
        averagePose = averagePose.plus(new Transform3d(new Pose3d(), getUpdatedIndiviualCameraEstimatedPose(leftCamPoseEstimator)));
        // Divide by number of poses (3) to get the average pose
        averagePose = averagePose.div(3);

        estimatedRobotPose = averagePose;

     }

     private Transform3d[] getUpdatedIndividualCameraVisibleToteAprilTags(PhotonCamera camera, Transform3d fromRobot, Transform3d[] curList) {
        Transform3d[] localVisibleToteAprilTags = curList;
        // For every visible target...
        for(PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
            // If its ID is less than 12...
            if (target.getFiducialId() < 12) {
                localVisibleToteAprilTags[target.getFiducialId()] = 
                // If there isnt already a value for that april tag...
                localVisibleToteAprilTags[target.getFiducialId()] == null ? 
                // Set the index of its ID to the transform from the bot to the target
                target.getBestCameraToTarget().plus(fromRobot) : 
                // Otherwise set the index of its ID to the average of the value that is there and the transform from the bot to the target
                localVisibleToteAprilTags[target.getFiducialId()].plus(target.getBestCameraToTarget().plus(fromRobot)).div(2);
            }
        }
        return localVisibleToteAprilTags;
     }

     public void updateVisibleToteAprilTags() {
        // Reset the list of t10ote april tags to an empty one
        visibleToteAprilTags = new Transform3d[12];

        visibleToteAprilTags = getUpdatedIndividualCameraVisibleToteAprilTags(frontCam, frontCamFromRobot, visibleToteAprilTags);
        visibleToteAprilTags = getUpdatedIndividualCameraVisibleToteAprilTags(rightCam, rightCamFromRobot, visibleToteAprilTags);
        visibleToteAprilTags = getUpdatedIndividualCameraVisibleToteAprilTags(leftCam, leftCamFromRobot, visibleToteAprilTags);
     }
}
