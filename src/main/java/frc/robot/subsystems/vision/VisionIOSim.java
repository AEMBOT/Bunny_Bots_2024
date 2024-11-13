package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;


public class VisionIOSim implements VisionIO {

    private final VisionSystemSim visionSim;

    // Front Cam Sim
    private final PhotonCameraSim frontCam;
    private final PhotonPoseEstimator frontPoseEstimator;
    
    private Optional<Pose3d> estimatedRobotPose = Optional.of(new Pose3d());

    public VisionIOSim() {
        PhotonCamera front = new PhotonCamera("front");

        frontPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            front, frontCamToRobot);

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagFieldLayout);

        SimCameraProperties frontCamProps = new SimCameraProperties();
        frontCamProps.setCalibration(frontCamHorizRes, frontCamVertRes, frontCamRotation);
        frontCamProps.setCalibError(frontCamPxErr, frontCamPxErrStdDev);
        frontCamProps.setFPS(frontCamFPS);
        frontCamProps.setAvgLatencyMs(frontCamAvgLatency);
        frontCamProps.setLatencyStdDevMs(frontCamLatencyStdDev);
        
        frontCam = new PhotonCameraSim(front, frontCamProps);

        visionSim.addCamera(frontCam, frontCamToRobot);

        frontCam.enableDrawWireframe(true);

    }

    public void updateInputs(VisionIOInputs inputs) {
        updateEstimatedPose();
        inputs.estimatedRobotPose = estimatedRobotPose;
    }

    public void updatePose(Pose2d pose) {
        visionSim.update(pose);
    }

    public void updateEstimatedPose() {
        //Checks if the pose estimator has a value
        frontPoseEstimator.update().ifPresentOrElse(
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
