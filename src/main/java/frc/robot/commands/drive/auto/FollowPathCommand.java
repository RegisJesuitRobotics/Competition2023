package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.types.rich.Pose2dEntry;
import frc.robot.telemetry.types.rich.TrajectoryEntry;
import frc.robot.utils.RaiderUtils;
import java.util.function.Supplier;

public class FollowPathCommand extends CommandBase {
    private static final Pose2dEntry desiredPoseEntry =
            new Pose2dEntry("/followPath/desiredPose", MiscConstants.TUNING_MODE);
    private static final TrajectoryEntry trajectoryEntry =
            new TrajectoryEntry("/followPath/trajectory", MiscConstants.TUNING_MODE);

    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<PathPlannerTrajectory> pathSupplier;
    private PathPlannerTrajectory currentPath;

    // We don't use a profiled PID controller for the angle because the path should
    // already profile it for us
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            AutoConstants.TRANSLATION_POSITION_GAINS.createLoggablePIDController("/followPath/xController"),
            AutoConstants.TRANSLATION_POSITION_GAINS.createLoggablePIDController("/followPath/yController"),
            AutoConstants.ANGULAR_POSITION_PID_GAINS.createLoggablePIDController("/followPath/thetaController"));

    private final Timer timer = new Timer();

    public FollowPathCommand(PathPlannerTrajectory path, SwerveDriveSubsystem driveSubsystem) {
        this(path, true, driveSubsystem);
    }

    public FollowPathCommand(PathPlannerTrajectory path, boolean shouldFlipIfRed, SwerveDriveSubsystem driveSubsystem) {
        this(
                () -> {
                    if (shouldFlipIfRed && RaiderUtils.shouldFlip()) {
                        return RaiderUtils.allianceFlip(path);
                    }
                    return path;
                },
                driveSubsystem);
    }

    /**
     * A follow path command made with the trajectory
     *
     * @param pathSupplier the trajectory
     * @param driveSubsystem the swerve drive subsystem
     */
    public FollowPathCommand(Supplier<PathPlannerTrajectory> pathSupplier, SwerveDriveSubsystem driveSubsystem) {
        this.pathSupplier = pathSupplier;
        this.driveSubsystem = driveSubsystem;

        driveController.setTolerance(new Pose2d(0.0254, 0.0254, Rotation2d.fromRotations(1)));

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentPath = pathSupplier.get();

        // TODO: Validate, does rotation matter?
        if (currentPath
                        .getInitialHolonomicPose()
                        .minus(driveSubsystem.getPose())
                        .getTranslation()
                        .getNorm()
                > Units.inchesToMeters(6.0)) {
            driveSubsystem.resetOdometry(currentPath.getInitialHolonomicPose());
        }

        driveSubsystem.getField2d().getObject("traj").setTrajectory(currentPath);
        trajectoryEntry.append(currentPath);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        Pose2d currentPose = driveSubsystem.getPose();

        PathPlannerState desiredState = (PathPlannerState) currentPath.sample(currentTime);
        ChassisSpeeds chassisSpeeds = driveController.calculate(currentPose, desiredState);

        Pose2d desiredPose = new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation);

        desiredPoseEntry.append(desiredPose);
        driveSubsystem.getField2d().getObject("followPathDesired").setPose(desiredPose);

        driveSubsystem.setChassisSpeeds(chassisSpeeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(currentPath.getTotalTimeSeconds()) && driveController.atReference();
    }
}
