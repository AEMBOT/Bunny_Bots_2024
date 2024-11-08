package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import java.util.ArrayList; 
import edu.wpi.first.math.geometry.Pose3d;
public class VisionIOReal implements VisionIO {
    
    private final PhotonCamera frontCam;
    private final PhotonPoseEstimator frontCamPoseEstimator;

    private ArrayList<Pose3d> visibleAprilTags = new ArrayList<Pose3d>();

    public VisionIOReal() {
        // Front Cam
        frontCam = new PhotonCamera(frontCamName);
        frontCamPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCam,
            frontCamToRobot);
    }

    public void updateInputs(VisionIOInputs inputs) {}

    // Returns the pose of the robot estimated by its vision
    public void updateEstimatedPose() {
        frontCamPoseEstimator.update();    
    }
}
