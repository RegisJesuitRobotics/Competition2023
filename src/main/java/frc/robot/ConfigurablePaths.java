package frc.robot;

// TODO:MAX_ACCELERATION

import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.paths.WaypointsCommandPair;
import frc.robot.utils.trajectory.HolonomicTrajectory;
import frc.robot.utils.trajectory.HolonomicTrajectoryGenerator;
import frc.robot.utils.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.List;

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

    private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.MAX_PATH_VELOCITY_METERS_SECOND,
            AutoConstants.MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED);

    public ConfigurablePaths(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addMapValues();
        putDashboardTables();
    }

    private void putDashboardTables() {
        SmartDashboard.putData("/paths/start", startPositionChooser);
        SmartDashboard.putData("/paths/aroundCharger", aroundCharger);
        SmartDashboard.putData("/paths/firstPiece", firstPiece);
        SmartDashboard.putData("/paths/firstTarget", firstTarget);
        SmartDashboard.putData("/paths/secondPiece", secondPiece);
        SmartDashboard.putData("/paths/secondTarget", secondTarget);
        SmartDashboard.putData("/paths/balance", balance);
    }

    public SequentialCommandGroup generatePath() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        List<WaypointsCommandPair> waypoints = new ArrayList<>();
        waypoints.add(startPositionChooser.getSelected());
        waypoints.add(aroundCharger.getSelected());
        waypoints.add(firstPiece.getSelected());
        waypoints.add(aroundCharger.getSelected().reversed());
        waypoints.add(firstTarget.getSelected());

        waypoints.add(aroundCharger.getSelected());
        waypoints.add(secondPiece.getSelected());
        waypoints.add(aroundCharger.getSelected().reversed());
        waypoints.add(secondTarget.getSelected());
        // TODO: add charger translation
        System.out.println(waypoints);
        WaypointsCommandPair balancePair = balance.getSelected();
        if (balancePair == null) {
            waypoints.add(aroundCharger.getSelected());
        }

        List<Waypoint> currentTrajectoryPoints = new ArrayList<>();
        Field2d field = driveSubsystem.getField2d();
        int currentI = 0;
        for (WaypointsCommandPair waypointsCommandPair : waypoints) {
            List<Waypoint> currentWaypoints = waypointsCommandPair.getWaypoints();
            currentTrajectoryPoints.addAll(currentWaypoints);
            if (waypointsCommandPair.getCommand() != null) {
                HolonomicTrajectory holonomicTrajectory =
                        HolonomicTrajectoryGenerator.generate(trajectoryConfig, currentTrajectoryPoints);
                field.getObject("traj" + currentI).setTrajectory(holonomicTrajectory.trajectory());
                currentI++;
                currentTrajectoryPoints.clear();
                currentTrajectoryPoints.add(currentWaypoints.get(currentWaypoints.size() - 1));
                command.addCommands(
                        new FollowPathCommand(holonomicTrajectory, driveSubsystem), waypointsCommandPair.getCommand());
            }
        }

        if (currentTrajectoryPoints.size() > 0) {
            HolonomicTrajectory holonomicTrajectory =
                    HolonomicTrajectoryGenerator.generate(trajectoryConfig, currentTrajectoryPoints);
            field.getObject("traj" + currentI).setTrajectory(holonomicTrajectory.trajectory());
            currentTrajectoryPoints.clear();
        }

        return command;
    }

    private void addMapValues() {
        // TODO: finish these

        startPositionChooser.setDefaultOption(
                "1",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(Community.regionCorners[0], new Rotation2d(0))));
        startPositionChooser.addOption(
                "2",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(
                        Community.regionCorners[0]
                                .plus(Community.regionCorners[1])
                                .div(2),
                        new Rotation2d(0))));
        startPositionChooser.addOption(
                "3",
                new WaypointsCommandPair(Waypoint.fromHolonomicPose(Community.regionCorners[1], new Rotation2d(0))));

        for (int i = 0; i < 4; i++) {
            firstPiece.setDefaultOption(
                    String.valueOf(i + 1),
                    new WaypointsCommandPair(
                            Waypoint.fromHolonomicPose(StagingLocations.translations[i], new Rotation2d(0)),
                            new WaitCommand(0.5)));
            secondPiece.setDefaultOption(
                    String.valueOf(i + 1),
                    new WaypointsCommandPair(
                            Waypoint.fromHolonomicPose(StagingLocations.translations[i], new Rotation2d(0)),
                            new WaitCommand(0.5)));
        }

        for (int i = 0; i < 9; i += 3) {
            firstTarget.setDefaultOption(
                    String.valueOf(i + 1),
                    new WaypointsCommandPair(
                            Waypoint.fromHolonomicPose(Grids.lowTranslations[i], Rotation2d.fromDegrees(180)),
                            new WaitCommand(0.5)));
            secondTarget.setDefaultOption(
                    String.valueOf(i + 1),
                    new WaypointsCommandPair(
                            Waypoint.fromHolonomicPose(Grids.lowTranslations[i], Rotation2d.fromDegrees(180)),
                            new WaitCommand(0.5)));
        }

        aroundCharger.setDefaultOption(
                "right",
                new WaypointsCommandPair(List.of(
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[0].minus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(90.0)),
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[0].plus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(90.0)))));
        aroundCharger.setDefaultOption(
                "left",
                new WaypointsCommandPair(List.of(
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[1].minus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(-90.0)),
                        Waypoint.fromHolonomicPose(
                                Community.betweenChargingAndWall[1].plus(new Translation2d(1.0, 0.0)),
                                Rotation2d.fromDegrees(-90.0)))));

        balance.setDefaultOption(
                "true",
                new WaypointsCommandPair(List.of(Waypoint.fromHolonomicPose(
                        ((Community.chargingStationCorners[0]
                                .plus(Community.chargingStationCorners[1])
                                .div(2))),
                        new Rotation2d(0)))));
        balance.addOption("false", null);
    }
}
