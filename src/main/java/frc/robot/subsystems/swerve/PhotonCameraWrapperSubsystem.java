package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Optional;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;


public class PhotonCameraWrapperSubsystem extends SubsystemBase {
    //add camera names
    private PhotonCamera frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    private PhotonCamera backCamera = new PhotonCamera(BACK_CAMERA_NAME);

    private AprilTagFieldLayout fieldLayout;

    private RobotPoseEstimator poseEstimator;

    private ArrayList camList;

    {
        try {
            fieldLayout = new AprilTagFieldLayout(
                    (Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

            fieldLayout = new AprilTagFieldLayout((Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();


        camList.add(new Pair<PhotonCamera, Transform3d>(frontCamera, FRONT_CAMERA_LOCATION));
        camList.add(new Pair<PhotonCamera, Transform3d>(backCamera, BACK_CAMERA_LOCATION));

        poseEstimator = new RobotPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camList);
        
    }

    public Pair<Pose2d, Double> getVisionPose(Pose2d prevPosition) {
        poseEstimator.setReferencePose(prevPosition);

        double currentTime = Timer.getFPGATimestamp();

        Optional<Pair<Pose3d, Double>> result = poseEstimator.update();

        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(
                    result.get().getFirst().toPose2d(),
                    currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
