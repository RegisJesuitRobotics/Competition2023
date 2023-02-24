package frc.robot;

import static frc.robot.Constants.AutoScoreConstants.preScoreFromLocations;
import static frc.robot.Constants.AutoScoreConstants.scoreFromLocations;
import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.commands.HomeHomeableCommand;
import frc.robot.commands.PositionClawCommand;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawState;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.intake.FlipperSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.paths.WaypointsCommandPair;
import frc.robot.utils.trajectory.HolonomicTrajectory;
import frc.robot.utils.trajectory.HolonomicTrajectoryGenerator;
import frc.robot.utils.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class ConfigurablePaths {

    private final ListenableSendableChooser<List<WaypointsCommandPair>> startPositionChooser =
            new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> aroundCharger = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> firstPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<List<WaypointsCommandPair>> firstTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> secondPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<List<WaypointsCommandPair>> secondTarget =
            new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> balance = new ListenableSendableChooser<>();

    private final SwerveDriveSubsystem driveSubsystem;
    private final LiftSubsystem liftSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final FlipperSubsystem flipperSubsystem;

    private SequentialCommandGroup currentCommand;
    private int currentConfigHash = 0;

    public ConfigurablePaths(
            SwerveDriveSubsystem driveSubsystem,
            LiftSubsystem liftSubsystem,
            ExtensionSubsystem extensionSubsystem,
            ClawSubsystem clawSubsystem,
            FlipperSubsystem flipperSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.extensionSubsystem = extensionSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.flipperSubsystem = flipperSubsystem;
        addMapValues();
        putDashboardTables();
    }

    private void putDashboardTables() {
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/startPositionChooser", startPositionChooser);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/aroundCharger", aroundCharger);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/firstPiece", firstPiece);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/firstTarget", firstTarget);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/secondPiece", secondPiece);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/secondTarget", secondTarget);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/balance", balance);
    }

    private int getHash() {
        return Objects.hash(
                startPositionChooser.getSelected(),
                aroundCharger.getSelected(),
                firstPiece.getSelected(),
                firstTarget.getSelected(),
                secondPiece.getSelected(),
                secondTarget.getSelected(),
                balance.getSelected());
    }

    public void generatePath() {
        currentCommand = new SequentialCommandGroup(Commands.parallel(
                new HomeHomeableCommand(LiftConstants.HOME_VOLTAGE, LiftConstants.HOME_CURRENT, liftSubsystem),
                new HomeHomeableCommand(
                        ExtensionConstants.HOME_VOLTAGE, ExtensionConstants.HOME_CURRENT, extensionSubsystem)));
        currentConfigHash = getHash();

        WaypointsCommandPair aroundChargerPair = aroundCharger.getSelected();

        WaypointsCommandPair firstPiecePair = firstPiece.getSelected();
        List<WaypointsCommandPair> firstTargetPair = firstTarget.getSelected();

        WaypointsCommandPair secondPiecePair = secondPiece.getSelected();
        List<WaypointsCommandPair> secondTargetPair = secondTarget.getSelected();

        List<WaypointsCommandPair> waypoints = new ArrayList<>(startPositionChooser.getSelected());
        if (firstPiecePair != null) {
            waypoints.add(aroundChargerPair);
            waypoints.add(firstPiecePair);
            if (firstTargetPair != null) {
                waypoints.add(aroundChargerPair.reversed());
                waypoints.addAll(firstTargetPair);

                // First piece is a prerequisite for the second piece
                if (secondPiecePair != null) {
                    waypoints.add(aroundChargerPair);
                    waypoints.add(secondPiecePair);
                    if (secondTargetPair != null) {
                        waypoints.add(aroundChargerPair.reversed());
                        waypoints.addAll(secondTargetPair);
                    }
                }
            }
        }

        // TODO: What if we don't score first piece, then we slam right into charging station if we have it selected
        // TODO: add charger translation
        WaypointsCommandPair balancePair = balance.getSelected();
        if (balancePair == null) {
            waypoints.add(aroundCharger.getSelected());
        }

        List<Waypoint> currentTrajectoryPoints = new ArrayList<>();
        Field2d field = driveSubsystem.getField2d();
        boolean firstRun = true;
        int currentTrajectoryIndex = 0;
        for (WaypointsCommandPair waypointsCommandPair : waypoints) {
            List<Waypoint> currentWaypoints = waypointsCommandPair.getWaypoints();
            currentTrajectoryPoints.addAll(currentWaypoints);
            if (waypointsCommandPair.getCommand() != null) {
                // If the first point has a command, then add it but don't generate a trajectory with one point
                if (!firstRun) {
                    HolonomicTrajectory holonomicTrajectory = HolonomicTrajectoryGenerator.generate(
                            AutoConstants.TRAJECTORY_CONSTRAINTS, currentTrajectoryPoints);
                    field.getObject("traj" + currentTrajectoryIndex).setTrajectory(holonomicTrajectory.trajectory());
                    currentTrajectoryIndex++;
                    currentTrajectoryPoints.clear();
                    currentTrajectoryPoints.add(currentWaypoints.get(currentWaypoints.size() - 1));
                    currentCommand.addCommands(new FollowPathCommand(holonomicTrajectory, driveSubsystem));
                }
                currentCommand.addCommands(new ProxyCommand(waypointsCommandPair.getCommand()));
            }
            firstRun = false;
        }

        if (currentTrajectoryPoints.size() > 0) {
            HolonomicTrajectory holonomicTrajectory = HolonomicTrajectoryGenerator.generate(
                    AutoConstants.TRAJECTORY_CONSTRAINTS, currentTrajectoryPoints);
            field.getObject("traj" + currentTrajectoryIndex).setTrajectory(holonomicTrajectory.trajectory());
            currentTrajectoryPoints.clear();
        }
    }

    public SequentialCommandGroup getCurrentCommandAndUpdateIfNeeded() {
        if (getHash() != currentConfigHash) {
            DataLogManager.log("Generating path in get");
            generatePath();
        }
        return currentCommand;
    }

    private void addMapValues() {
        addScoringOptions(startPositionChooser, false);

        addPieceOptions(firstPiece);
        addPieceOptions(secondPiece);

        addScoringOptions(firstTarget, true);
        addScoringOptions(secondTarget, true);

        aroundCharger.setDefaultOption(
                "Right",
                new WaypointsCommandPair(List.of(
                        new Waypoint(Community.betweenChargingAndWall[0].minus(new Translation2d(2.0, 0.0))),
                        new Waypoint(Community.betweenChargingAndWall[0].plus(new Translation2d(1.0, 0.0))))));
        aroundCharger.addOption(
                "Left",
                new WaypointsCommandPair(List.of(
                        new Waypoint(Community.betweenChargingAndWall[1].minus(new Translation2d(2.0, 0.0))),
                        new Waypoint(Community.betweenChargingAndWall[1].plus(new Translation2d(1.0, 0.0))))));

        // TODO
        balance.setDefaultOption("True", null);
        balance.addOption("False", null);
    }

    static final Translation2d gamePieceOffset = new Translation2d(Units.inchesToMeters(-9.0), 0.0);
    static final Pose2d[] gamePiecePickUpLocations = new Pose2d[StagingLocations.translations.length];

    static {
        for (int i = 0; i < gamePiecePickUpLocations.length; i++) {
            gamePiecePickUpLocations[i] =
                    new Pose2d(StagingLocations.translations[i].plus(gamePieceOffset), Rotation2d.fromDegrees(0.0));
        }
    }

    private Command getPickupSequence() {
        return new WaitCommand(0.5);
        //        return Commands.sequence(
        //                Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE)),
        //                new PositionClawCommand(AutoScoreConstants.LOW, liftSubsystem, extensionSubsystem));
    }

    private void addPieceOptions(ListenableSendableChooser<WaypointsCommandPair> sendableChooser) {
        sendableChooser.setDefaultOption("None", null);
        sendableChooser.addOption(
                "Far Right",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(gamePiecePickUpLocations[0]), getPickupSequence()));
        sendableChooser.addOption(
                "Middle Right",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(gamePiecePickUpLocations[1]), getPickupSequence()));
        sendableChooser.addOption(
                "Middle Left",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(gamePiecePickUpLocations[2]), getPickupSequence()));
        sendableChooser.addOption(
                "Far Left",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(gamePiecePickUpLocations[3]), getPickupSequence()));
    }

    private Command getScoreSequence() {
        return Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.OPEN), clawSubsystem);
    }

    private Command getPreScoreSequence() {
        return Commands.parallel(
                new PositionClawCommand(AutoScoreConstants.HIGH, liftSubsystem, extensionSubsystem),
                Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE), clawSubsystem));
    }

    private Command getPostScoreSequence() {
        return Commands.parallel(
                new PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem),
                Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.OPEN), clawSubsystem));
    }

    private List<WaypointsCommandPair> getPairListFromI(int i) {
        return List.of(
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(preScoreFromLocations[i]), getPreScoreSequence()),
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[i]), getScoreSequence()),
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(preScoreFromLocations[i]), getPostScoreSequence()));
    }

    private void addScoringOptions(
            ListenableSendableChooser<List<WaypointsCommandPair>> sendableChooser, boolean allowNone) {

        if (allowNone) {
            sendableChooser.setDefaultOption("None", null);
            sendableChooser.addOption(
                    "Cone 1",
                    List.of(new WaypointsCommandPair(
                            Waypoint.fromHolonomicPose(preScoreFromLocations[0]), getPreScoreSequence())));
        } else {
            sendableChooser.setDefaultOption("Cone 1", getPairListFromI(0));
        }
        sendableChooser.addOption("Cube 2", getPairListFromI(1));
        sendableChooser.addOption("Cone 3", getPairListFromI(2));
        sendableChooser.addOption("Cone 4", getPairListFromI(3));
        sendableChooser.addOption("Cube 5", getPairListFromI(4));
        sendableChooser.addOption("Cone 6", getPairListFromI(5));
        sendableChooser.addOption("Cone 7", getPairListFromI(6));
        sendableChooser.addOption("Cube 8", getPairListFromI(7));
        sendableChooser.addOption("Cone 9", getPairListFromI(8));
    }
}
