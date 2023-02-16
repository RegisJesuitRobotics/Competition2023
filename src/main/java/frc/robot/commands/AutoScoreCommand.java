package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.AutoScoreConstants.ScoreLevel;
import frc.robot.Constants.AutoScoreConstants.ScorePiece;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.lift.PositionClawCommand;
import frc.robot.subsystems.LiftExtensionSuperStructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderUtils;
import frc.robot.utils.trajectory.HolonomicTrajectory;
import frc.robot.utils.trajectory.HolonomicTrajectoryGenerator;
import frc.robot.utils.trajectory.ObstacleAvoidanceZones;
import frc.robot.utils.trajectory.Waypoint;
import java.util.List;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(
            ScoreLevel scoreLevel,
            IntSupplier scorePositionSupplier,
            SwerveDriveSubsystem driveSubsystem,
            LiftExtensionSuperStructure liftExtensionSuperStructure) {
        Supplier<HolonomicTrajectory> trajectoryGeneratorSupplier = () -> {
            Pose2d desiredPose = AutoScoreConstants.scoreFromLocations[scorePositionSupplier.getAsInt()];
            List<Waypoint> waypoints = List.of(
                    Waypoint.fromHolonomicPose(RaiderUtils.flipIfShould(driveSubsystem.getPose())),
                    Waypoint.fromHolonomicPose(desiredPose));
            waypoints = ObstacleAvoidanceZones.addIntermediaryWaypoints(waypoints);
            return HolonomicTrajectoryGenerator.generate(AutoConstants.TRAJECTORY_CONSTRAINTS, waypoints);
        };
        addCommands(Commands.parallel(
                new FollowPathCommand(trajectoryGeneratorSupplier, driveSubsystem),
                new ProxyCommand(() -> new PositionClawCommand(
                        getScoreClawTranslation(scoreLevel, getScorePiece(scorePositionSupplier.getAsInt())),
                        liftExtensionSuperStructure))));
    }

    public static Translation2d getScoreClawTranslation(ScoreLevel level, ScorePiece piece) {
        switch (level) {
            case LOW -> {
                return piece == ScorePiece.CUBE ? AutoScoreConstants.CUBE_LOW : AutoScoreConstants.CONE_LOW;
            }
            case MID -> {
                return piece == ScorePiece.CUBE ? AutoScoreConstants.CUBE_MID : AutoScoreConstants.CONE_MID;
            }
            case HIGH -> {
                return piece == ScorePiece.CUBE ? AutoScoreConstants.CUBE_HIGH : AutoScoreConstants.CONE_HIGH;
            }
            default -> throw new IllegalArgumentException("Bad level: " + level);
        }
    }

    public static ScorePiece getScorePiece(int scorePosition) {
        if (scorePosition == 1 || scorePosition == 4 || scorePosition == 7) {
            return ScorePiece.CUBE;
        }
        return ScorePiece.CONE;
    }
}
