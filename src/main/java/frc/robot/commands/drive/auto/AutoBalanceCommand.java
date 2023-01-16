package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutoBalanceCommand extends SequentialCommandGroup {
    public AutoBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        addCommands(new FollowPathCommand(
                () -> {
                    // FIXME
                    Pose2d currentPose = driveSubsystem.getPose();
                    Translation2d beforeStationPoint =
                            FieldConstants.CENTER_OF_CHARGE_STATION.minus(new Translation2d(1.0, 0.0));
                    Translation2d toStationPoint = currentPose.getTranslation().minus(beforeStationPoint);
                    Translation2d finalPoint = FieldConstants.CENTER_OF_CHARGE_STATION;
                    Translation2d toFinalPoint = toStationPoint.minus(finalPoint);
                    return PathPlanner.generatePath(
                            AutoConstants.PATH_CONSTRAINTS,
                            new PathPoint(
                                    currentPose.getTranslation(), toStationPoint.getAngle(), currentPose.getRotation()),
                            new PathPoint(beforeStationPoint, toFinalPoint.getAngle(), Rotation2d.fromDegrees(0.0)),
                            new PathPoint(finalPoint, toFinalPoint.getAngle(), Rotation2d.fromDegrees(0.0)));
                },
                driveSubsystem),
                new AutoBalanceCommand(driveSubsystem),
                new LockModulesCommand(driveSubsystem));
    }
}
