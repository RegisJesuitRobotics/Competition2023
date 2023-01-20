package frc.robot.subsystems.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapperSubsystem extends SubsystemBase {
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<PhotonPoseEstimator>();

    public PhotonCameraWrapperSubsystem() {
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = new AprilTagFieldLayout(
                    (Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME), VisionConstants.FRONT_CAMERA_LOCATION));
        poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, new PhotonCamera(VisionConstants.BACK_CAMERA_NAME), VisionConstants.BACK_CAMERA_LOCATION));
    }

    public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        for (PhotonPoseEstimator poseEstimator : poseEstimators) {
            poseEstimator.setReferencePose(prevEstimatedRobotPose);
        }
        List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
        for (PhotonPoseEstimator poseEstimator : poseEstimators) {
            Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
            estimatedRobotPose.ifPresent(updatedPoses::add);
        }
        return updatedPoses;
    }
}
