package frc.robot;

import static frc.robot.Constants.AutoScoreConstants.scoreFromLocations;
import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.auto.FollowPathCommand;
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

    private final ListenableSendableChooser<WaypointsCommandPair> startPositionChooser =
            new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> aroundCharger = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> firstPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> firstTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> secondPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> secondTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<WaypointsCommandPair> balance = new ListenableSendableChooser<>();

    private final SwerveDriveSubsystem driveSubsystem;

    private SequentialCommandGroup currentCommand;
    private int currentConfigHash = 0;

    public ConfigurablePaths(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
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
        currentCommand = new SequentialCommandGroup();
        currentConfigHash = getHash();

        List<WaypointsCommandPair> waypoints = new ArrayList<>();

        WaypointsCommandPair aroundChargerPair = aroundCharger.getSelected();

        WaypointsCommandPair firstPiecePair = firstPiece.getSelected();
        WaypointsCommandPair firstTargetPair = firstTarget.getSelected();

        WaypointsCommandPair secondPiecePair = secondPiece.getSelected();
        WaypointsCommandPair secondTargetPair = secondTarget.getSelected();

        waypoints.add(startPositionChooser.getSelected());
        if (firstPiecePair != null) {
            waypoints.add(aroundChargerPair);
            waypoints.add(firstPiecePair);
            if (firstTargetPair != null) {
                waypoints.add(aroundChargerPair.reversed());
                waypoints.add(firstTargetPair);

                // First piece is a prerequisite for the second piece
                if (secondPiecePair != null) {
                    waypoints.add(aroundChargerPair);
                    waypoints.add(secondPiecePair);
                    if (secondTargetPair != null) {
                        waypoints.add(aroundChargerPair.reversed());
                        waypoints.add(secondTargetPair);
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
        // TODO: finish these

        // WaitCommand to simulate placing preload
        addScoringOptions(startPositionChooser, false);

        addPieceOptions(firstPiece);
        addPieceOptions(secondPiece);

        addScoringOptions(firstTarget, true);
        addScoringOptions(secondTarget, true);

        aroundCharger.setDefaultOption(
                "Right",
                new WaypointsCommandPair(List.of(
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[0].minus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(90.0)),
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[0].plus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(90.0)))));
        aroundCharger.addOption(
                "Left",
                new WaypointsCommandPair(List.of(
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[1].minus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(-90.0)),
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[1].plus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(-90.0)))));

        // TODO
        balance.setDefaultOption("True", null);
        balance.addOption("False", null);
    }

    static final Translation2d gamePieceOffset = new Translation2d(-0.2, 0.0);
    static final Pose2d[] gamePiecePickUpLocations = new Pose2d[StagingLocations.translations.length];

    static {
        for (int i = 0; i < gamePiecePickUpLocations.length; i++) {
            gamePiecePickUpLocations[i] =
                    new Pose2d(StagingLocations.translations[i].plus(gamePieceOffset), Rotation2d.fromDegrees(0.0));
        }
    }

    private void addPieceOptions(ListenableSendableChooser<WaypointsCommandPair> sendableChooser) {
        sendableChooser.setDefaultOption("None", null);
        sendableChooser.addOption(
                "Far Right",
                new WaypointsCommandPair(
                        Waypoint.fromHolonomicPose(gamePiecePickUpLocations[0]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Middle Right",
                new WaypointsCommandPair(
                        Waypoint.fromHolonomicPose(gamePiecePickUpLocations[1]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Middle Left",
                new WaypointsCommandPair(
                        Waypoint.fromHolonomicPose(gamePiecePickUpLocations[2]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Far Left",
                new WaypointsCommandPair(
                        Waypoint.fromHolonomicPose(gamePiecePickUpLocations[3]), new WaitCommand(0.75)));
    }

    private void addScoringOptions(ListenableSendableChooser<WaypointsCommandPair> sendableChooser, boolean allowNone) {
        if (allowNone) {
            sendableChooser.setDefaultOption("None", null);
            sendableChooser.addOption(
                    "Cone 1",
                    new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[0]), new WaitCommand(0.75)));
        } else {
            sendableChooser.setDefaultOption(
                    "Cone 1",
                    new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[0]), new WaitCommand(0.75)));
        }
        sendableChooser.addOption(
                "Cube 2",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[1]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cone 3",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[2]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cone 4",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[3]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cube 5",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[4]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cone 6",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[5]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cone 7",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[6]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cube 8",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[7]), new WaitCommand(0.75)));
        sendableChooser.addOption(
                "Cone 9",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(scoreFromLocations[8]), new WaitCommand(0.75)));
    }
}
