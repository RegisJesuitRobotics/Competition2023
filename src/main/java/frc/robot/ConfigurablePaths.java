package frc.robot;

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
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.intake.FlipperSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.paths.AutoAction;
import frc.robot.utils.paths.AutoAction.AutoActionCommands;
import frc.robot.utils.paths.AutoAction.PositionNeed;
import frc.robot.utils.trajectory.HolonomicTrajectory;
import frc.robot.utils.trajectory.HolonomicTrajectoryGenerator;
import frc.robot.utils.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class ConfigurablePaths {
    private static final String NONE = "None";
    private static final String WALL_SIDE = "Wall Side";
    private static final String HUMAN_PLAYER_SIDE = "Human Player Side";
    private static final String MOVE_OUTSIDE_COMMUNITY_HUMAN_PLAYER_SIDE = "Move Outside Community Human Player Side";
    private static final String MOVE_OUTSIDE_COMMUNITY_WALL_SIDE = "Move Outside Community Wall Side";
    private static final String BALANCE_FROM_HUMAN_PLAYER_SIDE_INSIDE_COMMUNITY =
            "Balance From Human Player Side (Inside Community)";
    private static final String BALANCE_FROM_CENTER_INSIDE_COMMUNITY = "Balance From Center (Inside Community)";
    private static final String BALANCE_FROM_WALL_SIDE_INSIDE_COMMUNITY = "Balance From Wall Side (Inside Community)";
    private static final String BALANCE_FROM_HUMAN_PLAYER_SIDE_OUTSIDE_COMMUNITY =
            "Balance From Human Player Side (Outside Community)";
    private static final String BALANCE_FROM_WALL_SIDE_OUTSIDE_COMMUNITY = "Balance From Wall Side (Outside Community)";
    private static final String FAR_HUMAN_PLAYER_SIDE = "Far Human Player Side";
    private static final String MIDDLE_HUMAN_PLAYER_SIDE = "Middle Human Player Side";
    private static final String FAR_WALL_SIDE = "Far Wall Side";
    private static final String MIDDLE_WALL_SIDE = "Middle Wall Side";
    private static final String[] SCORE_LOCATION_NAMES = new String[] {
        "Cone 1 (Wall Side)",
        "Cone 2",
        "Cone 3",
        "Cone 4",
        "Cone 5",
        "Cone 6",
        "Cone 7",
        "Cone 8",
        "Cone 9 (Human Player Side)",
    };

    private static final int MAX_TRAJECTORIES = 11;

    private final ListenableSendableChooser<List<AutoAction>> startPositionChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<AutoAction> aroundCharger = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<AutoAction> firstPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<List<AutoAction>> firstTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<AutoAction> secondPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<List<AutoAction>> secondTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<AutoAction> finalAction = new ListenableSendableChooser<>();

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
        addChoosersToDashboard();
        addPresetsToDashboard();
    }

    private void addChoosersToDashboard() {
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/startPositionChooser", startPositionChooser);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/aroundCharger", aroundCharger);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/firstPiece", firstPiece);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/firstTarget", firstTarget);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/secondPiece", secondPiece);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/secondTarget", secondTarget);
        SendableTelemetryManager.getInstance().addSendable("/autoChooser/finalAction", finalAction);
    }

    private void addPresetsToDashboard() {
        CommandBase humanPlayerSideGetOneThanBalancePreset = Commands.runOnce(() -> {
                    startPositionChooser.setSelected(SCORE_LOCATION_NAMES[7]);
                    aroundCharger.setSelected(HUMAN_PLAYER_SIDE);
                    firstPiece.setSelected(FAR_HUMAN_PLAYER_SIDE);
                    firstTarget.setSelected(SCORE_LOCATION_NAMES[6]);
                    secondPiece.setSelected(NONE);
                    secondTarget.setSelected(NONE);
                    finalAction.setSelected(BALANCE_FROM_HUMAN_PLAYER_SIDE_INSIDE_COMMUNITY);
                    generatePath();
                })
                .ignoringDisable(true)
                .withName("humanPlayerSideGetOneThanBalancePreset");
        SendableTelemetryManager.getInstance()
                .addSendable(
                        "/autoChooser/presets/humanPlayerSideGetOneThanBalancePreset",
                        humanPlayerSideGetOneThanBalancePreset);

        CommandBase wallSideGetOneThanBalancePreset = Commands.runOnce(() -> {
                    startPositionChooser.setSelected(SCORE_LOCATION_NAMES[1]);
                    aroundCharger.setSelected(WALL_SIDE);
                    firstPiece.setSelected(FAR_WALL_SIDE);
                    firstTarget.setSelected(SCORE_LOCATION_NAMES[2]);
                    secondPiece.setSelected(NONE);
                    secondTarget.setSelected(NONE);
                    finalAction.setSelected(BALANCE_FROM_WALL_SIDE_INSIDE_COMMUNITY);
                    generatePath();
                })
                .ignoringDisable(true)
                .withName("wallSideGetOneThanBalancePreset");
        SendableTelemetryManager.getInstance()
                .addSendable("/autoChooser/presets/wallSideGetOneThanBalancePreset", wallSideGetOneThanBalancePreset);

        CommandBase placeMiddleThanBalancePreset = Commands.runOnce(() -> {
                    startPositionChooser.setSelected(SCORE_LOCATION_NAMES[4]);
                    aroundCharger.setSelected(WALL_SIDE);
                    firstPiece.setSelected(NONE);
                    firstTarget.setSelected(NONE);
                    secondPiece.setSelected(NONE);
                    secondTarget.setSelected(NONE);
                    finalAction.setSelected(BALANCE_FROM_CENTER_INSIDE_COMMUNITY);
                    generatePath();
                })
                .ignoringDisable(true)
                .withName("placeMiddleThanBalancePreset");
        SendableTelemetryManager.getInstance()
                .addSendable("/autoChooser/presets/placeMiddleThanBalancePreset", placeMiddleThanBalancePreset);
    }

    private int getHash() {
        return Objects.hash(
                startPositionChooser.getSelected(),
                aroundCharger.getSelected(),
                firstPiece.getSelected(),
                firstTarget.getSelected(),
                secondPiece.getSelected(),
                secondTarget.getSelected(),
                finalAction.getSelected());
    }

    public void generatePath() {
        //        currentCommand = new SequentialCommandGroup(Commands.parallel(
        //                HomeCommandFactory.homeLiftCommand(liftSubsystem),
        //                HomeCommandFactory.homeExtensionCommand(extensionSubsystem)));
        currentCommand = new SequentialCommandGroup(new WaitCommand(0.2));
        currentConfigHash = getHash();

        AutoAction aroundChargerPair = aroundCharger.getSelected();

        AutoAction firstPiecePair = firstPiece.getSelected();
        List<AutoAction> firstTargetPair = firstTarget.getSelected();

        AutoAction secondPiecePair = secondPiece.getSelected();
        List<AutoAction> secondTargetPair = secondTarget.getSelected();

        List<AutoAction> waypoints = new ArrayList<>(startPositionChooser.getSelected());

        // Tracks the robot location at the current path that is being used
        boolean insideCommunity = true;

        if (firstPiecePair != null) {
            waypoints.add(aroundChargerPair);
            waypoints.add(firstPiecePair);
            insideCommunity = false;
            if (firstTargetPair != null) {
                waypoints.add(aroundChargerPair.reversed());
                waypoints.addAll(firstTargetPair);
                insideCommunity = true;

                // First piece is a prerequisite for the second piece
                if (secondPiecePair != null) {
                    waypoints.add(aroundChargerPair);
                    waypoints.add(secondPiecePair);
                    insideCommunity = false;
                    if (secondTargetPair != null) {
                        waypoints.add(aroundChargerPair.reversed());
                        waypoints.addAll(secondTargetPair);
                        insideCommunity = true;
                    }
                }
            }
        }

        // TODO: What if we don't score first piece, then we slam right into charging station if we have it selected
        // TODO: add charger translation
        AutoAction balancePair = finalAction.getSelected();
        if (balancePair != null) {
            if (balancePair.getNeed() == PositionNeed.INSIDE_COMMUNITY && !insideCommunity) {
                waypoints.add(aroundChargerPair.reversed());
            } else if (balancePair.getNeed() == PositionNeed.OUTSIDE_COMMUNITY && insideCommunity) {
                waypoints.add(aroundChargerPair);
            }
            waypoints.add(balancePair);
        }

        List<Waypoint> currentTrajectoryPoints = new ArrayList<>();
        Field2d field = driveSubsystem.getField2d();
        boolean firstRun = true;
        int currentTrajectoryIndex = 0;
        for (AutoAction autoAction : waypoints) {
            List<Waypoint> currentWaypoints = autoAction.getWaypoints();
            currentTrajectoryPoints.addAll(currentWaypoints);
            Command whileDrivingCommand = autoAction.getCommands().getWhileDrivingCommand();
            Command whileDrivingNoWaitCommand = autoAction.getCommands().getWhileDrivingNoWait();
            Command stoppedCommand = autoAction.getCommands().getStoppedCommand();
            if (stoppedCommand != null || whileDrivingCommand != null || whileDrivingNoWaitCommand != null) {
                // If the first point has a command, then add it but don't generate a trajectory with one point
                if (!firstRun) {
                    HolonomicTrajectory holonomicTrajectory = HolonomicTrajectoryGenerator.generate(
                            AutoConstants.TRAJECTORY_CONSTRAINTS, currentTrajectoryPoints);
                    field.getObject("traj" + currentTrajectoryIndex).setTrajectory(holonomicTrajectory.trajectory());
                    currentTrajectoryIndex++;
                    currentTrajectoryPoints.clear();
                    //                     Add the last point of the previous trajectory to the start of the next
                    // trajectory
                    currentTrajectoryPoints.add(currentWaypoints.get(currentWaypoints.size() - 1));

                    Command pathCommand = new FollowPathCommand(holonomicTrajectory, driveSubsystem);
                    if (whileDrivingCommand != null) {
                        pathCommand = pathCommand.alongWith(whileDrivingCommand);
                    }
                    if (whileDrivingNoWaitCommand != null) {
                        pathCommand = pathCommand.deadlineWith(whileDrivingNoWaitCommand);
                    }

                    currentCommand.addCommands(pathCommand);
                }
                if (stoppedCommand != null) {
                    currentCommand.addCommands(stoppedCommand);
                }
            }
            firstRun = false;
        }

        if (currentTrajectoryPoints.size() > 1) {
            HolonomicTrajectory holonomicTrajectory = HolonomicTrajectoryGenerator.generate(
                    AutoConstants.TRAJECTORY_CONSTRAINTS, currentTrajectoryPoints);
            field.getObject("traj" + currentTrajectoryIndex).setTrajectory(holonomicTrajectory.trajectory());
            currentTrajectoryIndex++;

            currentCommand.addCommands(new FollowPathCommand(holonomicTrajectory, driveSubsystem));
            currentTrajectoryPoints.clear();
        }

        // Remove all trajectories from field2d that may be left over from previous runs
        while (currentTrajectoryIndex < MAX_TRAJECTORIES) {
            field.getObject("traj" + currentTrajectoryIndex).setPoses(List.of());
            currentTrajectoryIndex++;
        }
    }

    public SequentialCommandGroup getCurrentCommandAndUpdateIfNeeded() {
        if (getHash() != currentConfigHash) {
            DataLogManager.log("Generating path in get");
            generatePath();
        }
        return currentCommand;
    }

    private static final double balanceXOffset = Units.inchesToMeters(3.0);
    private static final double preInsideCommunityBalanceX =
            Community.chargingStationInnerX - (MiscConstants.FULL_ROBOT_WIDTH_METERS / 2.0) - balanceXOffset;
    private static final double preOutsideCommunityBalanceX =
            Community.chargingStationOuterX + (MiscConstants.FULL_ROBOT_WIDTH_METERS / 2.0) + balanceXOffset;
    private static final double balanceX =
            (Community.chargingStationOuterX - Community.chargingStationInnerX) / 2.0 + Community.chargingStationInnerX;
    private static final double balanceLeftRightYOffset = Units.inchesToMeters(6.0);
    private static final double leftBalanceY =
            Community.chargingStationLeftY - (MiscConstants.FULL_ROBOT_LENGTH_METERS / 2.0) - balanceLeftRightYOffset;
    private static final double rightBalanceY =
            Community.chargingStationRightY + (MiscConstants.FULL_ROBOT_LENGTH_METERS / 2.0) + balanceLeftRightYOffset;
    private static final double centerBalanceY =
            (Community.chargingStationLeftY - Community.chargingStationRightY) / 2.0 + Community.chargingStationRightY;
    private static final Rotation2d balanceHolonomicRotation = Rotation2d.fromDegrees(-90.0);

    private void addMapValues() {
        addScoringOptions(startPositionChooser, false);

        addPieceOptions(firstPiece);
        addPieceOptions(secondPiece);

        addScoringOptions(firstTarget, true);
        addScoringOptions(secondTarget, true);

        aroundCharger.setDefaultOption(
                WALL_SIDE,
                new AutoAction(List.of(
                        new Waypoint(Community.betweenChargingAndWall[0].minus(new Translation2d(2.2, 0.0))),
                        new Waypoint(Community.betweenChargingAndWall[0].plus(new Translation2d(1.0, 0.0))))));
        aroundCharger.addOption(
                HUMAN_PLAYER_SIDE,
                new AutoAction(List.of(
                        new Waypoint(Community.betweenChargingAndWall[1].minus(new Translation2d(2.2, 0.0))),
                        new Waypoint(Community.betweenChargingAndWall[1].plus(new Translation2d(1.0, 0.0))))));

        finalAction.setDefaultOption(NONE, null);
        finalAction.addOption(
                MOVE_OUTSIDE_COMMUNITY_HUMAN_PLAYER_SIDE,
                new AutoAction(
                        List.of(
                                new Waypoint(Community.betweenChargingAndWall[1].minus(new Translation2d(2.0, 0.0))),
                                new Waypoint(Community.betweenChargingAndWall[1].plus(new Translation2d(1.0, 0.0)))),
                        PositionNeed.INSIDE_COMMUNITY));
        finalAction.addOption(
                MOVE_OUTSIDE_COMMUNITY_WALL_SIDE,
                new AutoAction(
                        List.of(
                                new Waypoint(Community.betweenChargingAndWall[0].minus(new Translation2d(2.0, 0.0))),
                                new Waypoint(Community.betweenChargingAndWall[0].plus(new Translation2d(1.0, 0.0)))),
                        PositionNeed.INSIDE_COMMUNITY));

        finalAction.addOption(
                BALANCE_FROM_HUMAN_PLAYER_SIDE_INSIDE_COMMUNITY,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(preInsideCommunityBalanceX, leftBalanceY),
                                        balanceHolonomicRotation),
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(balanceX, leftBalanceY), balanceHolonomicRotation)),
                        PositionNeed.INSIDE_COMMUNITY));
        finalAction.addOption(
                BALANCE_FROM_CENTER_INSIDE_COMMUNITY,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(preInsideCommunityBalanceX, centerBalanceY),
                                        balanceHolonomicRotation),
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(balanceX, centerBalanceY), balanceHolonomicRotation)),
                        PositionNeed.INSIDE_COMMUNITY));
        finalAction.addOption(
                BALANCE_FROM_WALL_SIDE_INSIDE_COMMUNITY,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(preInsideCommunityBalanceX, rightBalanceY),
                                        balanceHolonomicRotation),
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(balanceX, rightBalanceY), balanceHolonomicRotation)),
                        PositionNeed.INSIDE_COMMUNITY));

        finalAction.addOption(
                BALANCE_FROM_HUMAN_PLAYER_SIDE_OUTSIDE_COMMUNITY,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(preOutsideCommunityBalanceX, leftBalanceY),
                                        balanceHolonomicRotation),
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(balanceX, leftBalanceY), balanceHolonomicRotation)),
                        PositionNeed.OUTSIDE_COMMUNITY));
        finalAction.addOption(
                BALANCE_FROM_WALL_SIDE_OUTSIDE_COMMUNITY,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(preOutsideCommunityBalanceX, rightBalanceY),
                                        balanceHolonomicRotation),
                                Waypoint.fromHolonomicPose(
                                        new Translation2d(balanceX, rightBalanceY), balanceHolonomicRotation)),
                        PositionNeed.OUTSIDE_COMMUNITY));
    }

    private static final Translation2d gamePieceOffset = new Translation2d(Units.inchesToMeters(-9.0), 0.0);
    private static final Pose2d[] gamePiecePickUpLocations = new Pose2d[StagingLocations.translations.length];
    private static final Translation2d preGamePieceOffset = new Translation2d(Units.inchesToMeters(-30.0), 0.0);
    private static final Pose2d[] preGamePiecePickUpLocations = new Pose2d[StagingLocations.translations.length];

    static {
        for (int i = 0; i < gamePiecePickUpLocations.length; i++) {
            gamePiecePickUpLocations[i] =
                    new Pose2d(StagingLocations.translations[i].plus(gamePieceOffset), Rotation2d.fromDegrees(0.0));
            preGamePiecePickUpLocations[i] =
                    new Pose2d(StagingLocations.translations[i].plus(preGamePieceOffset), Rotation2d.fromDegrees(0.0));
        }
    }

    private AutoActionCommands getPickupSequence() {
        //        return new StoppedUnStoppedCommands.Builder().withStoppedCommand(
        //                () -> Commands.sequence(Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE)),
        // new PositionClawCommand(AutoScoreConstants.CARRY, liftSubsystem, extensionSubsystem))).build();
        return new AutoActionCommands.Builder()
                .withStoppedCommand(() -> new WaitCommand(0.3))
                .build();
    }

    private void addPieceOptions(ListenableSendableChooser<AutoAction> sendableChooser) {
        sendableChooser.setDefaultOption(NONE, null);
        sendableChooser.addOption(
                FAR_HUMAN_PLAYER_SIDE,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(preGamePiecePickUpLocations[3]),
                                Waypoint.fromHolonomicPose(gamePiecePickUpLocations[3])),
                        getPickupSequence()));
        sendableChooser.addOption(
                MIDDLE_HUMAN_PLAYER_SIDE,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(preGamePiecePickUpLocations[2]),
                                Waypoint.fromHolonomicPose(gamePiecePickUpLocations[2])),
                        getPickupSequence()));
        sendableChooser.addOption(
                MIDDLE_WALL_SIDE,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(preGamePiecePickUpLocations[1]),
                                Waypoint.fromHolonomicPose(gamePiecePickUpLocations[1])),
                        getPickupSequence()));
        sendableChooser.addOption(
                FAR_WALL_SIDE,
                new AutoAction(
                        List.of(
                                Waypoint.fromHolonomicPose(preGamePiecePickUpLocations[0]),
                                Waypoint.fromHolonomicPose(gamePiecePickUpLocations[0])),
                        getPickupSequence()));
    }

    private AutoActionCommands getScoreSequence() {
        //        return new StoppedUnStoppedCommands.Builder()
        //                .withStoppedCommand(() -> Commands.sequence(
        //                        Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.OPEN), clawSubsystem),
        //                        new WaitCommand(0.3)))
        //                .withWhileDrivingCommandNoWait(
        //                        () -> new SetExtensionPositionCommand(AutoScoreConstants.STOW.getSecond(),
        // extensionSubsystem))
        //                .build();

        return new AutoActionCommands.Builder()
                .withStoppedCommand(() -> new WaitCommand(0.3))
                .withWhileDrivingCommandNoWait(() -> new WaitCommand(0.5))
                .build();
    }

    private AutoActionCommands getPreScoreSequence() {
        //        return new StoppedUnStoppedCommands.Builder().withStoppedCommand(
        //                () -> MiscCommandsFactory.clawClearOfMidCubeCommand(liftSubsystem,
        // extensionSubsystem)).withWhileDrivingCommandWithWait(() -> Commands.parallel(
        //                new PositionClawCommand(AutoScoreConstants.HIGH, liftSubsystem, extensionSubsystem),
        //                Commands.runOnce(() -> clawSubsystem.setClawState(ClawState.CLOSE), clawSubsystem)
        //        )).build();
        return new AutoActionCommands.Builder()
                .withStoppedCommand(() -> new WaitCommand(0.3))
                .withWhileDrivingCommandWithWait(() -> new WaitCommand(0.5))
                .build();
    }

    private AutoActionCommands getPostScoreSequence() {
        //        return new AutoActionCommands.Builder().withStoppedCommand(() -> new
        // PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem)).build();
        return new AutoActionCommands.Builder()
                .withStoppedCommand(() -> new WaitCommand(0.75))
                .build();
    }

    private List<AutoAction> getPairListFromI(int i) {
        return List.of(
                new AutoAction(
                        Waypoint.fromHolonomicPose(AutoScoreConstants.preScoreFromLocations[i]), getPreScoreSequence()),
                new AutoAction(
                        Waypoint.fromHolonomicPose(AutoScoreConstants.scoreFromLocations[i]), getScoreSequence()),
                new AutoAction(
                        Waypoint.fromHolonomicPose(AutoScoreConstants.preScoreFromLocations[i]),
                        getPostScoreSequence()));
    }

    private void addScoringOptions(ListenableSendableChooser<List<AutoAction>> sendableChooser, boolean allowNone) {

        if (allowNone) {
            sendableChooser.setDefaultOption(NONE, null);
        }
        for (int i = AutoScoreConstants.preScoreFromLocations.length - 1; i >= 1; i--) {
            sendableChooser.addOption(SCORE_LOCATION_NAMES[i], getPairListFromI(i));
        }
        if (allowNone) {
            sendableChooser.addOption(
                    "Cone 1 (Wall Side)",
                    List.of(new AutoAction(
                            Waypoint.fromHolonomicPose(AutoScoreConstants.preScoreFromLocations[0]),
                            getPreScoreSequence())));
        } else {
            sendableChooser.setDefaultOption("Cone 1 (Wall Side)", getPairListFromI(0));
        }
    }
}
