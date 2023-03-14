package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.HomeCommandFactory;
import frc.robot.commands.PositionClawCommand;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.commands.drive.auto.balance.CorrectBalanceAndLockCommand;
import frc.robot.commands.drive.characterize.DriveTestingCommand;
import frc.robot.commands.drive.characterize.DriveTrainSysIDCompatibleLoggerCommand;
import frc.robot.commands.drive.characterize.SteerTestingCommand;
import frc.robot.commands.extension.SetExtensionPositionCommand;
import frc.robot.commands.lift.SetLiftPositionCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawState;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.RaiderCommands;
import java.util.List;

public class Autos {
    private final SwerveDriveSubsystem driveSubsystem;
    private final LiftSubsystem liftSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final ClawSubsystem clawSubsystem;

    private final ListenableSendableChooser<Command> autoCommandChooser = new ListenableSendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    public Autos(
            SwerveDriveSubsystem driveSubsystem,
            LiftSubsystem liftSubsystem,
            ExtensionSubsystem extensionSubsystem,
            ClawSubsystem clawSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.extensionSubsystem = extensionSubsystem;
        this.clawSubsystem = clawSubsystem;

        autoCommandChooser.addOption("Nothing", null);
        autoCommandChooser.setDefaultOption("Only Home", homeBoth());
        autoCommandChooser.addOption("HP Side Place Get Place", humanPlayerPlaceGetPlace());
        autoCommandChooser.addOption("HP Side Place Balance", humanPlayerPlaceBalance());
        autoCommandChooser.addOption("Wall Side Place Balance", wallPlaceBalance());
        autoCommandChooser.addOption("Center Balance", centerBalance());
        autoCommandChooser.addOption("Center Place Balance", centerPlaceBalance());
        autoCommandChooser.addOption("HP Side Place Mobility", humanPlayerPlaceMobility());
        autoCommandChooser.addOption("Wall Side Place Mobility", wallPlaceMobility());

        if (MiscConstants.TUNING_MODE) {
            autoCommandChooser.addOption("SysIDLogger", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
            autoCommandChooser.addOption("GreaseGears", new GreaseGearsCommand(driveSubsystem));
            autoCommandChooser.addOption("DriveTestingCommand", new DriveTestingCommand(1.0, driveSubsystem));
            autoCommandChooser.addOption("SteerTesting", new SteerTestingCommand(driveSubsystem));
            autoCommandChooser.addOption("AutoBalanceTeSting", new CorrectBalanceAndLockCommand(driveSubsystem));
        }

        new Trigger(autoCommandChooser::hasNewValue)
                .onTrue(RaiderCommands.runOnceAllowDisable(
                                () -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .withName("AutoAlertChecker"));

        SendableTelemetryManager.getInstance().addSendable("/autoChooser/AutoChooser", autoCommandChooser);
    }

    private Command wallPlaceMobility() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("WallSidePM", AutoConstants.TRAJECTORY_CONSTRAINTS);
        return placeMobility(trajectoryList);
    }

    private Command humanPlayerPlaceMobility() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("HPSidePM", AutoConstants.TRAJECTORY_CONSTRAINTS);
        return placeMobility(trajectoryList);
    }

    private Command humanPlayerPlaceGetPlace() {
        List<PathPlannerTrajectory> trajectoryList = PathPlanner.loadPathGroup(
                "HPSidePGP",
                AutoConstants.TRAJECTORY_CONSTRAINTS,
                AutoConstants.TRAJECTORY_CONSTRAINTS,
                AutoConstants.SLOW_TRAJECTORY_CONSTRAINTS,
                AutoConstants.SLOW_TRAJECTORY_CONSTRAINTS);

        return placeGetPlace(trajectoryList);
    }

    private Command humanPlayerPlaceBalance() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("HPSidePB", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return placeBalance(trajectoryList);
    }

    private Command wallPlaceBalance() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("WallSidePB", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return placeBalance(trajectoryList);
    }

    private Command centerBalance() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("CenterB", AutoConstants.VERY_SLOW_TRAJECTORY_CONSTRAINTS);

        return sequence(
                homeBoth(), followPath(trajectoryList.get(0)), followPath(trajectoryList.get(1)), correctBalance());
    }

    private Command centerPlaceBalance() {
        List<PathPlannerTrajectory> centerPBPart1 =
                PathPlanner.loadPathGroup("CenterPBPart1", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return sequence(
                parallel(homeBoth(), closeClaw()),
                clawHigh(),
                followPath(centerPBPart1.get(0)),
                openClaw(),
                followPath(centerPBPart1.get(1)),
                balanceFromConventionLocation(centerPBPart1.get(1).getEndState().poseMeters));
    }

    private Command placeGetPlace(List<PathPlannerTrajectory> trajectoryList) {
        return sequence(
                parallel(homeBoth(), closeClaw()),
                sequence(
                        clawHigh(),
                        sequence(
                                waitUntil(() -> liftSubsystem.getArmAngle().getDegrees() > -10.0),
                                followPath(trajectoryList.get(0)))),
                openClaw(),
                waitSeconds(0.5),
                parallel(
                        followPath(trajectoryList.get(1)),
                        retractExtension(),
                        sequence(waitSeconds(0.5), retractLift())),
                closeClaw(),
                waitSeconds(0.5),
                parallel(sequence(waitSeconds(0.5), followPath(trajectoryList.get(2))), clawMid()),
                openClaw());
    }

    private Command balanceFromConventionLocation(Pose2d basePose) {
        return sequence(
                toPoint(new Pose2d(basePose.getTranslation(), Rotation2d.fromDegrees(90.0))),
                clawStow(),
                toPoint(new Pose2d(
                        basePose.getTranslation().plus(new Translation2d(2.0, 0.0)), Rotation2d.fromDegrees(90.0))),
                correctBalance());
    }

    private Command placeMobility(List<PathPlannerTrajectory> trajectoryList) {
        return sequence(
                parallel(homeBoth(), closeClaw()),
                clawHigh(),
                followPath(trajectoryList.get(0)),
                openClaw(),
                parallel(
                        followPath(trajectoryList.get(1)),
                        retractExtension(),
                        sequence(waitSeconds(0.5), retractLift())));
    }

    private Command placeBalance(List<PathPlannerTrajectory> trajectoryList) {
        return sequence(
                parallel(homeBoth(), closeClaw()),
                clawHigh(),
                followPath(trajectoryList.get(0)),
                openClaw(),
                parallel(
                        followPath(trajectoryList.get(1)),
                        retractExtension(),
                        sequence(waitSeconds(0.5), retractLift())),
                correctBalance());
    }

    private Command homeBoth() {
        return parallel(
                HomeCommandFactory.homeLiftCommand(liftSubsystem).unless(liftSubsystem::isHomed),
                HomeCommandFactory.homeExtensionCommand(extensionSubsystem).unless(extensionSubsystem::isHomed));
    }

    private Command toPoint(Pose2d point) {
        return new SimpleToPointCommand(point, driveSubsystem);
    }

    private Command openClaw() {
        return Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.OPEN), clawSubsystem);
    }

    private Command closeClaw() {
        return Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE), clawSubsystem);
    }

    private Command followPath(PathPlannerTrajectory trajectory) {
        return new FollowPathCommand(trajectory, driveSubsystem);
    }

    private Command clawHigh() {
        return new PositionClawCommand(AutoScoreConstants.HIGH, liftSubsystem, extensionSubsystem);
    }

    private Command clawMid() {
        return new PositionClawCommand(AutoScoreConstants.MID, liftSubsystem, extensionSubsystem);
    }

    private Command clawStow() {
        return new PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem);
    }

    private Command retractExtension() {
        return new SetExtensionPositionCommand(AutoScoreConstants.STOW.getSecond(), extensionSubsystem);
    }

    private Command retractLift() {
        return new SetLiftPositionCommand(AutoScoreConstants.STOW.getFirst(), liftSubsystem);
    }

    private Command correctBalance() {
        return new CorrectBalanceAndLockCommand(driveSubsystem);
    }

    public Command getSelectedAuto() {
        return autoCommandChooser.getSelected();
    }
}
