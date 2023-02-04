package frc.robot;

// TODO:MAX_ACCELERATION
import static frc.robot.Constants.DriveTrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED;
import static frc.robot.Constants.DriveTrainConstants.MAX_VELOCITY_METERS_SECOND;
import static frc.robot.FieldConstants.Community.*;
import static frc.robot.FieldConstants.Grids.*;
import static frc.robot.FieldConstants.StagingLocations.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.paths.WaypointCommand;
import frc.robot.utils.trajectory.HolonomicTrajectory;
import frc.robot.utils.trajectory.HolonomicTrajectoryGenerator;
import frc.robot.utils.trajectory.IntermediateTrajectory;
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

    private final double maxVelocitySecond = MAX_VELOCITY_METERS_SECOND;

    private final double maxAcceleration = MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED;

    private SwerveDriveSubsystem driveSubsystem;

    private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxVelocitySecond, maxAcceleration);

    private HolonomicTrajectoryGenerator generator = new HolonomicTrajectoryGenerator();

    public ConfigurablePaths(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        createChooserOptions();
        addMapValues();
        putDahsboardTables();
    }

    private void putDahsboardTables() {
        SmartDashboard.putData("/paths/start", startPositionChooser);
        SmartDashboard.putData("/paths/aroundCharger", aroundCharger);
        SmartDashboard.putData("/paths/firstPiece", firstPiece);
        SmartDashboard.putData("/paths/firstTarget", firstTarget);
        SmartDashboard.putData("/paths/secondPiece", secondPiece);
        SmartDashboard.putData("/paths/secondTarget", secondTarget);
        SmartDashboard.putData("/paths/balance", balance);
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
    // TODO: add commands
    private List<IntermediateTrajectory> generateWaypoints() {
        List<IntermediateTrajectory> trajectories = new ArrayList<>();
        List<WaypointCommand> waypoints = new ArrayList<>();
        waypoints.add(new WaypointCommand(startMap.get(startPositionChooser.getSelected())));
        waypoints.add(new WaypointCommand(chargerMap.get(aroundCharger.getSelected())));
        waypoints.add(new WaypointCommand(firstPieceMap.get(firstPiece.getSelected()), new InstantCommand()));
        waypoints.add(new WaypointCommand(chargerMap.get(aroundCharger.getSelected())));
        waypoints.add(new WaypointCommand(firstTargetMap.get(firstPiece.getSelected()), new InstantCommand()));
        waypoints.add(new WaypointCommand(chargerMap.get(aroundCharger.getSelected())));
        waypoints.add(new WaypointCommand(firstPieceMap.get(secondPiece.getSelected()), new InstantCommand()));
        waypoints.add(new WaypointCommand(chargerMap.get(aroundCharger.getSelected())));
        waypoints.add(new WaypointCommand(firstTargetMap.get(secondPiece.getSelected()), new InstantCommand()));
        // TODO: add charger translation
        waypoints.add(new WaypointCommand(balanceMap.get(balance.getSelected()), new InstantCommand()));

        for (int i = 0; i < 7; i++) {
            WaypointCommand first = waypoints.get(i);
            WaypointCommand second = waypoints.get(i + 1);
            if (first.getWaypoint() == null || second.getWaypoint() == null){
                break;
            }
            if (first.getWaypoint().getTranslation() != null
                    && second.getWaypoint().getTranslation() != null) {
                List<WaypointCommand> points = List.of(first, second);
                if (i == 0 || waypoints.get(i - 1).getCommand() != null)
                    trajectories.add(new IntermediateTrajectory(points, false));
            }
        }

        return trajectories;
    }

    public SequentialCommandGroup generatePath() {
        SequentialCommandGroup command = new SequentialCommandGroup();
        List<IntermediateTrajectory> trajectories = generateWaypoints();
        Field2d field = driveSubsystem.getField2d();

        for (int i = 0; i<trajectories.size(); i++){
            FieldObject2d fieldObject2d =  field.getObject("field"+String.valueOf(i));
            HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generate(trajectoryConfig, trajectories.get(i).getWaypoint());
            command.addCommands(new FollowPathCommand(trajectory, driveSubsystem));
            fieldObject2d.setTrajectory(trajectory.trajectory());
//            command.addCommands(trajectories.get(i).getCommand());
        }
        return command;
    }

    private void addMapValues() {
        // TODO: finish these

        startMap.put("1", new Waypoint(regionCorners[0], new Rotation2d(0), null));
        startMap.put("2", new Waypoint(regionCorners[0].plus(regionCorners[1]).div(2), new Rotation2d(0), null));
        startMap.put("3", new Waypoint(regionCorners[1], new Rotation2d(0), null));

        for (int i = 0; i < 4; i++) {
            firstPieceMap.put(String.valueOf(i + 1), new Waypoint(translations[i], new Rotation2d(0), null));
        }

        for (int i = 0; i < 9; i += 3) {
            firstTargetMap.put(String.valueOf(i + 1), new Waypoint(lowTranslations[i], new Rotation2d(0), null));
        }
        firstTargetMap.put("nothing", null);

        chargerMap.put("left", Waypoint.fromHolonomicPose(new Pose2d(chargingStationCorners[3], new Rotation2d(0))));
        chargerMap.put("right", new Waypoint(chargingStationCorners[2], new Rotation2d(0), null));

        balanceMap.put(
                "true",
                new Waypoint(
                        ((chargingStationCorners[0]
                                .plus(chargingStationCorners[1])
                                .div(2))),
                        new Rotation2d(0),
                        null));
        balanceMap.put("false", null);
    }
}
