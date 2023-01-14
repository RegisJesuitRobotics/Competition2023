package frc.robot.subsystems.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class PhotonCameraWrapperSubsystem extends SubsystemBase {
    private final RobotPoseEstimator poseEstimator;

    private final ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<>();

    public PhotonCameraWrapperSubsystem() {
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = new AprilTagFieldLayout(
                    (Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // TODO: add camera names
        PhotonCamera frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
        camList.add(new Pair<>(frontCamera, VisionConstants.FRONT_CAMERA_LOCATION));
        PhotonCamera backCamera = new PhotonCamera(VisionConstants.BACK_CAMERA_NAME);
        camList.add(new Pair<>(backCamera, VisionConstants.BACK_CAMERA_LOCATION));

        poseEstimator = new RobotPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camList);
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
