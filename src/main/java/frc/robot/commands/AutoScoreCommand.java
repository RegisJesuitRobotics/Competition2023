package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.AutoScoreConstants.ScoreLevel;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawState;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.intake.FlipperSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderUtils;
import java.util.function.BooleanSupplier;
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
        BooleanSupplier farEnoughBack = () ->
                RaiderUtils.flipIfShould(driveSubsystem.getPose()).getX() >= AutoScoreConstants.ROBOT_SCORING_X + 0.73;
        int scorePosition = scorePositionSupplier.getAsInt();
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE), clawSubsystem),
                        Commands.runOnce(flipperSubsystem::setInStowedPosition, flipperSubsystem),
                        new SimpleToPointCommand(
                                () -> RaiderUtils.flipIfShould(AutoScoreConstants.preScoreFromLocations[scorePosition]),
                                driveSubsystem),
                        Commands.waitUntil(farEnoughBack)
                                .andThen(new ProxyCommand(() -> new PositionClawCommand(
                                        getScoreClawTranslation(scoreLevel), liftSubsystem, extensionSubsystem)))),
                new SimpleToPointCommand(
                        () -> RaiderUtils.flipIfShould(AutoScoreConstants.scoreFromLocations[scorePosition]),
                        driveSubsystem),
                Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.OPEN)),
                new SimpleToPointCommand(
                        () -> RaiderUtils.flipIfShould(AutoScoreConstants.preScoreFromLocations[scorePosition]),
                        driveSubsystem),
                new PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem));
    }

    private static Pair<Rotation2d, Double> getScoreClawTranslation(ScoreLevel level) {
        switch (level) {
            case LOW -> {
                return AutoScoreConstants.HIGH;
            }
            case MID -> {
                return AutoScoreConstants.MID;
            }
            case HIGH -> {
                return AutoScoreConstants.LOW;
            }
            default -> throw new IllegalArgumentException("Bad level: " + level);
        }
    }
}
