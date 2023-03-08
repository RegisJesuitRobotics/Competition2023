package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.HomeCommandFactory;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.drive.auto.balance.CorrectBalanceAndLockCommand;
import frc.robot.commands.drive.characterize.DriveTestingCommand;
import frc.robot.commands.drive.characterize.DriveTrainSysIDCompatibleLoggerCommand;
import frc.robot.commands.drive.characterize.SteerTestingCommand;
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
        autoCommandChooser.addOption("Human Player Side Place Get Place Balance", humanPlayerPlaceGetPlaceBalance());
        autoCommandChooser.addOption("Wall Side Place Get Place Balance", wallPlaceGetPlaceBalance());
        autoCommandChooser.addOption("Human Player Side Place Balance", humanPlayerPlaceBalance());
        autoCommandChooser.addOption("Wall Side Place Balance", wallPlaceBalance());
        autoCommandChooser.addOption("Center Balance", centerBalance());
        autoCommandChooser.addOption("Center Place Balance", centerPlaceBalance());

        if (MiscConstants.TUNING_MODE) {
            autoCommandChooser.addOption("SysIDLogger", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
            autoCommandChooser.addOption("GreaseGears", new GreaseGearsCommand(driveSubsystem));
            autoCommandChooser.addOption("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
            autoCommandChooser.addOption("SteerTesting", new SteerTestingCommand(driveSubsystem));
        }

        new Trigger(autoCommandChooser::hasNewValue)
                .onTrue(RaiderCommands.runOnceAllowDisable(
                                () -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .withName("Auto Alert Checker"));

        SendableTelemetryManager.getInstance().addSendable("/autoChooser/AutoChooser", autoCommandChooser);
    }

    private Command humanPlayerPlaceGetPlaceBalance() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("HPSidePGPB", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return placeGetPlaceBalance(trajectoryList);
    }

    private Command wallPlaceGetPlaceBalance() {
        List<PathPlannerTrajectory> trajectoryList =
                PathPlanner.loadPathGroup("WallSidePGPB", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return placeGetPlaceBalance(trajectoryList);
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
                PathPlanner.loadPathGroup("CenterB", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return Commands.sequence(homeBoth(), followPath(trajectoryList.get(0)), correctBalance());
    }

    private Command centerPlaceBalance() {
        List<PathPlannerTrajectory> centerPBPart1 =
                PathPlanner.loadPathGroup("CenterPBPart1", AutoConstants.TRAJECTORY_CONSTRAINTS);
        List<PathPlannerTrajectory> centerPBPart2 =
                PathPlanner.loadPathGroup("CenterPBPart2", AutoConstants.TRAJECTORY_CONSTRAINTS);

        return Commands.sequence(
                Commands.parallel(homeBoth(), closeClaw()),
                clawHigh(),
                followPath(centerPBPart1.get(0)),
                openClaw(),
                followPath(centerPBPart1.get(1)),
                toPoint(centerPBPart2.get(0).getInitialHolonomicPose()),
                followPath(centerPBPart2.get(0)),
                correctBalance());
    }

    private Command placeGetPlaceBalance(List<PathPlannerTrajectory> trajectoryList) {
        return Commands.sequence(
                Commands.parallel(homeBoth(), closeClaw()),
                clawHigh(),
                followPath(trajectoryList.get(0)),
                openClaw(),
                Commands.parallel(followPath(trajectoryList.get(1)), Commands.sequence(retractExtension(), clawStow())),
                closeClaw(),
                Commands.waitSeconds(0.5),
                Commands.parallel(followPath(trajectoryList.get(2)), clawHigh()),
                openClaw(),
                Commands.parallel(followPath(trajectoryList.get(3)), retractExtension()),
                clawStow(),
                followPath(trajectoryList.get(4)),
                correctBalance());
    }

    private Command placeBalance(List<PathPlannerTrajectory> trajectoryList) {
        return Commands.sequence(
                Commands.parallel(homeBoth(), closeClaw()),
                clawHigh(),
                followPath(trajectoryList.get(0)),
                openClaw(),
                Commands.parallel(followPath(trajectoryList.get(1)), Commands.sequence(retractExtension())),
                clawStow(),
                followPath(trajectoryList.get(2)),
                correctBalance());
    }

    private Command homeBoth() {
        return Commands.parallel(
                HomeCommandFactory.homeLiftCommand(liftSubsystem),
                HomeCommandFactory.homeExtensionCommand(extensionSubsystem));
    }

    private Command toPoint(Pose2d point) {
        //        return new SimpleToPointCommand(point, driveSubsystem);
        return new WaitCommand(1.0);
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
        //        return new PositionClawCommand(AutoScoreConstants.HIGH, liftSubsystem, extensionSubsystem);
        return new WaitCommand(1.0);
    }

    private Command clawStow() {
        //        return new PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem);
        return new WaitCommand(1.0);
    }

    private Command retractExtension() {
        //        return new SetExtensionPositionCommand(Units.inchesToMeters(0.5), extensionSubsystem);
        return new WaitCommand(0.5);
    }

    private Command correctBalance() {
        return new CorrectBalanceAndLockCommand(driveSubsystem);
    }

    public Command getSelectedAuto() {
        return autoCommandChooser.getSelected();
    }
}
