package frc.robot.commands.drive.auto.balance;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutoBalanceCommand extends SequentialCommandGroup {
    public AutoBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        addCommands(
                new FollowPathCommand(
                        () -> {
                            // FIXME: Use field coordinates instead of relative ones
                            Pose2d currentPose = driveSubsystem.getPose();
                            Pose2d toFinalPoint = currentPose.plus(
                                    new Transform2d(new Translation2d(0.0, 1.5), Rotation2d.fromDegrees(0.0)));
                            Translation2d translation =
                                    currentPose.minus(toFinalPoint).getTranslation();
                            return PathPlanner.generatePath(
                                    AutoConstants.AUTO_BALANCE_PATH_CONSTRAINTS,
                                    new PathPoint(
                                            currentPose.getTranslation(),
                                            new Rotation2d(-translation.getX(), -translation.getY()),
                                            currentPose.getRotation()),
                                    new PathPoint(
                                            toFinalPoint.getTranslation(),
                                            new Rotation2d(-translation.getX(), -translation.getY()),
                                            toFinalPoint.getRotation()));
                        },
                        driveSubsystem),
                new CorrectBalanceCommand(driveSubsystem),
                new LockModulesCommand(driveSubsystem));
    }
}
