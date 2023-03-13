package frc.robot.subsystems.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.telemetry.types.rich.Pose3dArrayEntry;
import frc.robot.telemetry.types.rich.Pose3dEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSubsystem extends SubsystemBase {
    private final AprilTagFieldLayout fieldLayout;
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
    private final List<Pose3dEntry> estimatedPoseEntries = new ArrayList<>();
    private final Pose3dArrayEntry visionTargetEntries =
            new Pose3dArrayEntry("/photon/targets", MiscConstants.TUNING_MODE);
    private final Pose3dArrayEntry unusedVisionTargetEntries =
            new Pose3dArrayEntry("/photon/unusedTargets", MiscConstants.TUNING_MODE);
    private final List<PhotonCamera> cameras = new ArrayList<>();

    private final Alert cameraNotConnectedAlert =
            new Alert("AprilTag Camera is Not Powered or Not Connected", AlertType.ERROR);

    public PhotonSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        cameras.add(new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME));
        cameras.add(new PhotonCamera(VisionConstants.BACK_CAMERA_NAME));

        poseEstimators.add(new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP, cameras.get(0), VisionConstants.FRONT_CAMERA_LOCATION));
        poseEstimators.add(new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP, cameras.get(1), VisionConstants.BACK_CAMERA_LOCATION));

        for (int i = 0; i < poseEstimators.size(); i++) {
            estimatedPoseEntries.add(new Pose3dEntry("/photon/estimatedPoses/" + i, MiscConstants.TUNING_MODE));
        }
    }

    public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        Robot.startWNode("PhotonSubsystem#getEstimatedGlobalPose");
        List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
        List<Pose3d> targetPoses = new ArrayList<>();
        List<Pose3d> unusedTargetPoses = new ArrayList<>();
        for (int i = 0; i < poseEstimators.size(); i++) {
            PhotonPoseEstimator poseEstimator = poseEstimators.get(i);
            PhotonCamera photonCamera = cameras.get(i);
            PhotonPipelineResult result = photonCamera.getLatestResult();

            // Remove bad tags if only one, also add to our array
            for (int j = result.targets.size() - 1; j >= 0; j--) {
                PhotonTrackedTarget target = result.targets.get(j);
                boolean shouldUse = (result.targets.get(j).getPoseAmbiguity() < VisionConstants.POSE_AMBIGUITY_CUTOFF
                                || result.targets.size() > 1)
                        && result.targets
                                        .get(j)
                                        .getBestCameraToTarget()
                                        .getTranslation()
                                        .getNorm()
                                < VisionConstants.DISTANCE_CUTOFF;
                if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                    Pose3d tagPose =
                            fieldLayout.getTagPose(target.getFiducialId()).get();
                    if (shouldUse) {
                        targetPoses.add(tagPose);
                    } else {
                        unusedTargetPoses.add(tagPose);
                    }
                }
                if (!shouldUse) {
                    result.targets.remove(j);
                }
            }

            poseEstimator.setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
            if (estimatedRobotPose.isPresent()) {
                estimatedPoseEntries.get(i).append(estimatedRobotPose.get().estimatedPose);
                updatedPoses.add(estimatedRobotPose.get());
            }
        }
        visionTargetEntries.append(targetPoses);
        unusedVisionTargetEntries.append(unusedTargetPoses);

        Robot.endWNode();
        return updatedPoses;
    }

    @Override
    public void periodic() {
        Robot.startWNode("PhotonSubsystem#periodic");
        boolean allCamerasConnected = true;
        for (PhotonCamera camera : cameras) {
            allCamerasConnected &= camera.isConnected();
        }
        cameraNotConnectedAlert.set(!allCamerasConnected);
        Robot.endWNode();
    }
}
