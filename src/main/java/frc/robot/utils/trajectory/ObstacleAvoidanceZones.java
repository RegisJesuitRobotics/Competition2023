package frc.robot.utils.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MiscConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.utils.AdjacencyMatrixGraph;
import java.util.ArrayList;
import java.util.List;

public class ObstacleAvoidanceZones {
    record Rectangle(Translation2d lowerCorner, Translation2d upperCorner) {
        public boolean isInside(Translation2d point) {
            return point.getX() >= lowerCorner.getX()
                    && point.getX() <= upperCorner.getX()
                    && point.getY() >= lowerCorner.getY()
                    && point.getY() <= upperCorner.getY();
        }
    }

    private static final Rectangle innerCommunity = new Rectangle(
            new Translation2d(Community.innerX, Community.rightY),
            new Translation2d(Community.chargingStationInnerX, Community.leftY));
    private static final Rectangle rightChargingStation = new Rectangle(
            new Translation2d(Community.chargingStationInnerX, Community.rightY),
            new Translation2d(Community.chargingStationOuterX, Community.chargingStationRightY));
    private static final Rectangle leftChargingStation = new Rectangle(
            new Translation2d(Community.chargingStationInnerX, Community.chargingStationLeftY),
            new Translation2d(Community.chargingStationOuterX, Community.leftY));

    // IntelliJ says this is suspicious, but it's not cause of WPILib coordinate system
    @SuppressWarnings("SuspiciousNameCombination")
    private static final Rectangle noMansLand = new Rectangle(
            new Translation2d(Community.chargingStationOuterX, Community.rightY),
            new Translation2d(FieldConstants.fieldLength - Community.outerX, FieldConstants.fieldWidth));

    private static final List<Rectangle> zones =
            List.of(innerCommunity, rightChargingStation, leftChargingStation, noMansLand);

    private static final AdjacencyMatrixGraph<Rectangle, List<Waypoint>> graph = new AdjacencyMatrixGraph<>();

    static {
        final double longerRobotSide =
                Math.max(MiscConstants.FULL_ROBOT_WIDTH_METERS, MiscConstants.FULL_ROBOT_LENGTH_METERS);
        final double extraSpace = Units.inchesToMeters(3.0);
        final Rotation2d zero = Rotation2d.fromDegrees(0.0);
        addAutoReverseEdges(
                innerCommunity,
                rightChargingStation,
                List.of(
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationInnerX - (longerRobotSide / 2.0) - extraSpace,
                                        ((Community.chargingStationRightY - Community.rightY) / 2.0)
                                                + Community.rightY),
                                zero),
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationInnerX + extraSpace,
                                        ((Community.chargingStationRightY - Community.rightY) / 2.0)
                                                + Community.rightY),
                                zero)));
        addAutoReverseEdges(
                innerCommunity,
                leftChargingStation,
                List.of(
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationInnerX - (longerRobotSide / 2.0) - extraSpace,
                                        (Community.chargingStationLeftY - Community.leftY) / 2.0 + Community.leftY),
                                zero),
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationInnerX + extraSpace,
                                        (Community.chargingStationLeftY - Community.leftY) / 2.0 + Community.leftY),
                                zero)));
        addAutoReverseEdges(
                rightChargingStation,
                noMansLand,
                List.of(
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationOuterX - extraSpace,
                                        ((Community.chargingStationRightY - Community.rightY) / 2.0)
                                                + Community.rightY),
                                zero),
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationOuterX + (longerRobotSide / 2.0) + extraSpace,
                                        ((Community.chargingStationRightY - Community.rightY) / 2.0)
                                                + Community.rightY),
                                zero)));
        addAutoReverseEdges(
                leftChargingStation,
                noMansLand,
                List.of(
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationOuterX - extraSpace,
                                        (Community.chargingStationLeftY - Community.leftY) / 2.0 + Community.leftY),
                                zero),
                        Waypoint.fromDifferentialPose(
                                new Translation2d(
                                        Community.chargingStationOuterX + (longerRobotSide / 2.0) + extraSpace,
                                        (Community.chargingStationLeftY - Community.leftY) / 2.0 + Community.leftY),
                                zero)));
    }

    private static void addAutoReverseEdges(Rectangle to, Rectangle from, List<Waypoint> waypoints) {
        List<Waypoint> reversedWaypoints = new ArrayList<>();
        for (int i = waypoints.size() - 1; i >= 0; i--) {
            Waypoint waypoint = waypoints.get(i);
            if (waypoint.getDriveRotation().isPresent()) {
                waypoint = new Waypoint(
                        waypoint.getTranslation(),
                        waypoint.getDriveRotation().get().rotateBy(Rotation2d.fromDegrees(180.0)),
                        waypoint.getHolonomicRotation().orElse(null));
            }
            reversedWaypoints.add(waypoint);
        }
        graph.addEdge(to, from, waypoints, reversedWaypoints);
    }

    private static Rectangle getZone(Translation2d point) {
        for (Rectangle zone : zones) {
            if (zone.isInside(point)) {
                return zone;
            }
        }
        return null;
    }

    public static List<Waypoint> addIntermediaryWaypoints(List<Waypoint> waypoints) {
        List<Waypoint> returnedList = new ArrayList<>(waypoints);
        for (int i = 0; i < returnedList.size() - 1; i++) {
            Waypoint current = returnedList.get(i);
            Waypoint next = returnedList.get(i + 1);
            Rectangle currentZone = getZone(current.getTranslation());
            Rectangle nextZone = getZone(next.getTranslation());
            if (currentZone != null && nextZone != null && currentZone != nextZone) {
                List<List<Rectangle>> paths = graph.getPaths(currentZone, nextZone);
                double minDistance = Double.MAX_VALUE;
                List<Waypoint> bestPath = List.of();
                for (List<Rectangle> path : paths) {
                    List<Waypoint> pathPoints = new ArrayList<>();
                    pathPoints.add(current);
                    for (int j = 0; j < path.size() - 1; j++) {
                        pathPoints.addAll(graph.getEdge(path.get(j), path.get(j + 1)));
                    }

                    double distance = 0.0;
                    for (int j = 0; j < pathPoints.size() - 1; j++) {
                        distance += pathPoints
                                .get(j)
                                .getTranslation()
                                .getDistance(pathPoints.get(j + 1).getTranslation());
                    }
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestPath = pathPoints;
                    }
                }
                // Don't re-add the first and last points
                for (int j = 1; j < bestPath.size(); j++) {
                    returnedList.add(i + j, bestPath.get(j));
                }
                i += bestPath.size() - 1;
            }
        }
        return returnedList;
    }
}
