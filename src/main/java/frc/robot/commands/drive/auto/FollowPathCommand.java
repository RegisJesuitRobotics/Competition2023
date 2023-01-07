package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.Supplier;

public class FollowPathCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<PathPlannerTrajectory> pathSupplier;
    private PathPlannerTrajectory currentPath;

    // We don't use a profiled PID controller for the angle because the path should
    // already profile it for us
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/xController"),
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/yController"),
            AutoConstants.PATH_ANGULAR_POSITION_PID_GAINS.createLoggablePIDController("followPath/thetaController"));

    private final PPHolonomicDriveController nextDriveController = new PPHolonomicDriveController(
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/nextXController"),
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/nextYController"),
            AutoConstants.PATH_ANGULAR_POSITION_PID_GAINS.createLoggablePIDController(
                    "followPath/nextThetaController"));

    private final Timer timer = new Timer();

    public FollowPathCommand(PathPlannerTrajectory path, SwerveDriveSubsystem driveSubsystem) {
        this(() -> path, driveSubsystem);
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

        driveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromRotations(5.0)));

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentPath = pathSupplier.get();
        if (MiscConstants.TUNING_MODE) {
            PathPlannerServer.sendActivePath(currentPath.getStates());
        }
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

        PathPlannerState nextDesiredState = (PathPlannerState) currentPath.sample(currentTime + Constants.DT);
        Pose2d assumedNextPose = currentPose.plus(new Transform2d(
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).times(Constants.DT),
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond).times(Constants.DT)));
        driveSubsystem.getField2d().getObject("followPathNext").setPose(assumedNextPose);
        driveSubsystem.getField2d().getObject("followPathDesired").setPose(desiredPose);

        ChassisSpeeds nextChassisSpeeds = nextDriveController.calculate(assumedNextPose, nextDesiredState);

        driveSubsystem.setChassisSpeeds(chassisSpeeds, nextChassisSpeeds, false);

        if (MiscConstants.TUNING_MODE) {
            PathPlannerServer.sendPathFollowingData(
                    new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation), currentPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            driveSubsystem.stopMovement();
        }

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(currentPath.getTotalTimeSeconds()) && driveController.atReference();
    }
}
