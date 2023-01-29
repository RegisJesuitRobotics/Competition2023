package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED;
// TODO:MAX_ACCELERATION
import static frc.robot.Constants.DriveTrainConstants.MAX_VELOCITY_METERS_SECOND;
import static frc.robot.FieldConstants.Community.*;
import static frc.robot.FieldConstants.Grids.*;
import static frc.robot.FieldConstants.StagingLocations.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.trajectory.HolonomicTrajectory;
import frc.robot.utils.trajectory.HolonomicTrajectoryGenerator;
import frc.robot.utils.trajectory.Waypoint;
import java.util.*;

public class ConfigurablePaths {

    private final ListenableSendableChooser<String> startPositionChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<String> aroundCharger = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<String> firstPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<String> firstTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<String> secondPiece = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<String> secondTarget = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<String> balance = new ListenableSendableChooser<>();

    private final Map<String, Waypoint> startMap = new HashMap<>();
    private final Map<String, Waypoint> chargerMap = new HashMap<>();
    private final Map<String, Waypoint> firstPieceMap = new HashMap<>();
    private final Map<String, Waypoint> firstTargetMap = new HashMap<>();
    private final Map<String, Waypoint> secondPieceMap = new HashMap<>();
    private final Map<String, Waypoint> secondTargetMap = new HashMap<>();
    private final Map<String, Waypoint> balanceMap = new HashMap<>();

    public ConfigurablePaths() {
        createChooserOptions();
        addMapValues();
    }

    public HolonomicTrajectory generatePath() {
        TrajectoryConfig trajectoryConfig =
                new TrajectoryConfig(MAX_VELOCITY_METERS_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED);

        return HolonomicTrajectoryGenerator.generate(trajectoryConfig, generateWaypoints());
    }

    private void createChooserOptions() {
        for (int i = 1; i <= 3; i++) {
            startPositionChooser.addOption(String.valueOf(i), String.valueOf(i));
        }

        aroundCharger.addOption("left", "left");
        aroundCharger.addOption("right", "right");

        for (int i = 1; i <= 4; i++) {
            firstPiece.addOption(String.valueOf(i), String.valueOf(i));
        }

        for (int i = 1; i <= 5; i++) {
            firstTarget.addOption(String.valueOf(i), String.valueOf(i));
        }

        for (int i = 1; i <= 4; i++) {
            secondPiece.addOption(String.valueOf(i), String.valueOf(i));
        }

        for (int i = 1; i <= 5; i++) {
            secondTarget.addOption(String.valueOf(i), String.valueOf(i));
        }

        balance.addOption("true", "true");
        balance.addOption("false", "false");
    }

    private List<Waypoint> generateWaypoints() {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(startMap.get(startPositionChooser.getSelected()));
        waypoints.add(chargerMap.get(aroundCharger.getSelected()));
        waypoints.add(firstPieceMap.get(firstPiece.getSelected()));
        waypoints.add(chargerMap.get(aroundCharger.getSelected()));
        waypoints.add(firstTargetMap.get(firstPiece.getSelected()));
        waypoints.add(firstPieceMap.get(secondPiece.getSelected()));
        waypoints.add(firstTargetMap.get(secondPiece.getSelected()));
        waypoints.add(balanceMap.get(balance.getSelected()));

        return waypoints;
    }

    private void addMapValues() {
        // TODO: finish these

        for (int i = 0; i < 3; i++) {
            startMap.put(String.valueOf(i + 1), new Waypoint(regionCorners[i], new Rotation2d(0), null));
        }

        for (int i = 0; i < 4; i++) {
            firstPieceMap.put(String.valueOf(i + 1), new Waypoint(translations[i], new Rotation2d(0), null));
        }

        for (int i = 0; i < 9; i += 3) {
            firstTargetMap.put(String.valueOf(i + 1), new Waypoint(lowTranslations[i], new Rotation2d(0), null));
        }
        firstTargetMap.put("nothing", new Waypoint(null, null, null));

        chargerMap.put("left", new Waypoint(chargingStationCorners[3], new Rotation2d(0), null));
        chargerMap.put("right", new Waypoint(chargingStationCorners[2], new Rotation2d(0), null));

        balanceMap.put(
                "true",
                new Waypoint(
                        ((chargingStationCorners[0]
                                .plus(chargingStationCorners[1])
                                .div(2))),
                        new Rotation2d(0),
                        null));
        balanceMap.put("false", new Waypoint(null, null, null));
    }
}
