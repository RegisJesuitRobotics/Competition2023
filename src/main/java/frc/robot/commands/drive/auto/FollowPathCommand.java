package frc.robot.commands.drive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.trajectory.CustomHolonomicDriveController;
import frc.robot.utils.trajectory.HolonomicTrajectory;

import java.util.function.Supplier;

public class FollowPathCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<HolonomicTrajectory> pathSupplier;
    private HolonomicTrajectory currentPath;

    // We don't use a profiled PID controller for the angle because the path should
    // already profile it for us
    private final CustomHolonomicDriveController driveController = new CustomHolonomicDriveController(
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/xController"),
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/yController"),
            AutoConstants.PATH_ANGULAR_POSITION_PID_GAINS.createLoggablePIDController("followPath/thetaController"));

    private final CustomHolonomicDriveController nextDriveController = new CustomHolonomicDriveController(
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/nextXController"),
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/nextYController"),
            AutoConstants.PATH_ANGULAR_POSITION_PID_GAINS.createLoggablePIDController(
                    "followPath/nextThetaController"));

    private final Timer timer = new Timer();

    public FollowPathCommand(HolonomicTrajectory path, SwerveDriveSubsystem driveSubsystem) {
        this(() -> path, driveSubsystem);
    }

    /**
     * A follow path command made with the trajectory
     *
     * @param pathSupplier the trajectory
     * @param driveSubsystem the swerve drive subsystem
     */
    public FollowPathCommand(Supplier<HolonomicTrajectory> pathSupplier, SwerveDriveSubsystem driveSubsystem) {
        this.pathSupplier = pathSupplier;
        this.driveSubsystem = driveSubsystem;

        driveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromRotations(5.0)));

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentPath = pathSupplier.get();

        driveSubsystem.getField2d().getObject("traj").setTrajectory(currentPath.trajectory());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        Pose2d currentPose = driveSubsystem.getPose();

        HolonomicTrajectory.State state = currentPath.sample(currentTime);
        ChassisSpeeds chassisSpeeds = driveController.calculate(currentPose, state);

        Pose2d desiredPose = new Pose2d(state.poseState().poseMeters.getTranslation(), state.rotationState().position);

        HolonomicTrajectory.State nextDesiredState = currentPath.sample(currentTime + Constants.DT);
        Pose2d assumedNextPose = currentPose.plus(new Transform2d(
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).times(Constants.DT),
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond).times(Constants.DT)));
        driveSubsystem.getField2d().getObject("followPathNext").setPose(assumedNextPose);
        driveSubsystem.getField2d().getObject("followPathDesired").setPose(desiredPose);

        ChassisSpeeds nextChassisSpeeds = nextDriveController.calculate(assumedNextPose, nextDesiredState);

        driveSubsystem.setChassisSpeeds(chassisSpeeds, nextChassisSpeeds, false);
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
        return timer.hasElapsed(currentPath.trajectory().getTotalTimeSeconds()) && driveController.atReference();
    }
}
