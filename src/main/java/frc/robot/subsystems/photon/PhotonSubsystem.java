package frc.robot.subsystems.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.telemetry.types.rich.Pose3dArrayEntry;
import frc.robot.telemetry.types.rich.Pose3dEntry;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSubsystem extends SubsystemBase {
    private final AprilTagFieldLayout fieldLayout;
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
    private final List<Pose3dEntry> estimatedPoseEntries = new ArrayList<>();
    private final Pose3dArrayEntry visionTargetEntries =
            new Pose3dArrayEntry("/photon/targets", MiscConstants.TUNING_MODE);
    private final List<PhotonCamera> cameras = new ArrayList<>();

    public PhotonSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        cameras.add(new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME));

        poseEstimators.add(new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP, cameras.get(0), VisionConstants.FRONT_CAMERA_LOCATION));

        for (int i = 0; i < poseEstimators.size(); i++) {
            estimatedPoseEntries.add(new Pose3dEntry("/photon/estimatedPoses/" + i, MiscConstants.TUNING_MODE));
        }
    }

    public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
        List<Pose3d> targetPoses = new ArrayList<>();
        for (int i = 0; i < poseEstimators.size(); i++) {
            PhotonPoseEstimator poseEstimator = poseEstimators.get(i);
            poseEstimator.setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
            if (estimatedRobotPose.isPresent()) {
                updatedPoses.add(estimatedRobotPose.get());
                estimatedPoseEntries.get(i).append(estimatedRobotPose.get().estimatedPose);
            }
            PhotonCamera photonCamera = cameras.get(i);
            for (PhotonTrackedTarget target : photonCamera.getLatestResult().getTargets()) {
                if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                    targetPoses.add(
                            fieldLayout.getTagPose(target.getFiducialId()).get());
                }
            }
        }
        visionTargetEntries.append(targetPoses);
        return updatedPoses;
    }
}
