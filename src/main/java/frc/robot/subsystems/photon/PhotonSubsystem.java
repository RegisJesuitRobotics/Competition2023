package frc.robot.subsystems.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.telemetry.types.rich.Pose3dEntry;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonSubsystem extends SubsystemBase {
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
    private final List<Pose3dEntry> estimatedPoseEntries = new ArrayList<>();

    public PhotonSubsystem() {
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = new AprilTagFieldLayout(
                    (Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
            fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        poseEstimators.add(new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP,
                new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME),
                VisionConstants.FRONT_CAMERA_LOCATION));

        for (int i = 0; i < poseEstimators.size(); i++) {
            estimatedPoseEntries.add(new Pose3dEntry("/photon/estimatedPoses/" + i, MiscConstants.TUNING_MODE));
        }
    }

    public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
        for (int i = 0; i < poseEstimators.size(); i++) {
            PhotonPoseEstimator poseEstimator = poseEstimators.get(i);
            poseEstimator.setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
            if (estimatedRobotPose.isPresent()) {
                updatedPoses.add(estimatedRobotPose.get());
                estimatedPoseEntries.get(i).append(estimatedRobotPose.get().estimatedPose);
            }
        }
        return updatedPoses;
    }
}
