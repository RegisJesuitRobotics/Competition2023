package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.AutoScoreConstants.ScoreLevel;
import frc.robot.Constants.AutoScoreConstants.ScorePiece;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawState;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.intake.FlipperSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderUtils;
import java.util.function.IntSupplier;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(
            ScoreLevel scoreLevel,
            IntSupplier scorePositionSupplier,
            SwerveDriveSubsystem driveSubsystem,
            LiftSubsystem liftSubsystem,
            ExtensionSubsystem extensionSubsystem,
            ClawSubsystem clawSubsystem,
            FlipperSubsystem flipperSubsystem) {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE), clawSubsystem),
                        Commands.runOnce(flipperSubsystem::setInStowedPosition, flipperSubsystem),
                        new SimpleToPointCommand(
                                () -> RaiderUtils.flipIfShould(new Pose2d(
                                        AutoScoreConstants.scoreFromLocations[scorePositionSupplier.getAsInt()]
                                                .getTranslation()
                                                .plus(new Translation2d(0.5, 0.0)),
                                        AutoScoreConstants.scoreFromLocations[scorePositionSupplier.getAsInt()]
                                                .getRotation())),
                                driveSubsystem),
                        new ProxyCommand(() -> new PositionClawCommand(
                                getScoreClawTranslation(scoreLevel, getScorePiece(scorePositionSupplier.getAsInt())),
                                liftSubsystem,
                                extensionSubsystem))),
                new SimpleToPointCommand(
                        () -> RaiderUtils.flipIfShould(
                                AutoScoreConstants.scoreFromLocations[scorePositionSupplier.getAsInt()]),
                        driveSubsystem),
                Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.OPEN)));
    }

    private static Translation2d getScoreClawTranslation(ScoreLevel level, ScorePiece piece) {
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

    private static ScorePiece getScorePiece(int scorePosition) {
        if (scorePosition == 1 || scorePosition == 4 || scorePosition == 7) {
            return ScorePiece.CUBE;
        }
        return ScorePiece.CONE;
    }
}
