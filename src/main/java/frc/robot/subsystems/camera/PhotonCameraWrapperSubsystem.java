package frc.robot.subsystems.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class PhotonCameraWrapperSubsystem extends SubsystemBase {
    private final RobotPoseEstimator poseEstimator;

    public PhotonCameraWrapperSubsystem() {
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = new AprilTagFieldLayout(
                    (Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        poseEstimator = new RobotPoseEstimator(
                fieldLayout,
                PoseStrategy.AVERAGE_BEST_TARGETS,
                List.of(
                        new Pair<>(
                                new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME),
                                VisionConstants.FRONT_CAMERA_LOCATION),
                        new Pair<>(
                                new PhotonCamera(VisionConstants.BACK_CAMERA_NAME),
                                VisionConstants.BACK_CAMERA_LOCATION)));
    }

    public void setReferencePose(Pose2d referencePose) {
        poseEstimator.setReferencePose(referencePose);
    }

    public Optional<Pair<Pose3d, Double>> getVisionPose() {
        double currentTime = Timer.getFPGATimestamp();

        Optional<Pair<Pose3d, Double>> result = poseEstimator.update();

        return result.map(pose3dDoublePair ->
                new Pair<>(pose3dDoublePair.getFirst(), currentTime - pose3dDoublePair.getSecond()));
    }
}
