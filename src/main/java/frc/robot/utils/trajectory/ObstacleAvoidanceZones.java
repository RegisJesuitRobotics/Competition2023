package frc.robot.utils.trajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MiscConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.utils.AdjacencyMatrixGraphWithReversableList;
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

    private static final AdjacencyMatrixGraphWithReversableList<Rectangle, Translation2d> graph =
            new AdjacencyMatrixGraphWithReversableList<>();

    static {
        final double longerRobotSide =
                Math.max(MiscConstants.FULL_ROBOT_WIDTH_METERS, MiscConstants.FULL_ROBOT_LENGTH_METERS);
        final double extraSpace = Units.inchesToMeters(3.0);
        graph.addEdge(
                innerCommunity,
                rightChargingStation,
                List.of(
                        new Translation2d(
                                Community.chargingStationInnerX - (longerRobotSide / 2.0) - extraSpace,
                                (Community.chargingStationRightY - Community.rightY) / 2.0),
                        new Translation2d(
                                Community.chargingStationInnerX + extraSpace,
                                (Community.chargingStationRightY - Community.rightY) / 2.0)));
        graph.addEdge(
                innerCommunity,
                leftChargingStation,
                List.of(
                        new Translation2d(
                                Community.chargingStationInnerX - (longerRobotSide / 2.0) - extraSpace,
                                (Community.chargingStationLeftY - Community.leftY) / 2.0),
                        new Translation2d(
                                Community.chargingStationInnerX + extraSpace,
                                (Community.chargingStationLeftY - Community.leftY) / 2.0)));
        graph.addEdge(
                rightChargingStation,
                noMansLand,
                List.of(
                        new Translation2d(
                                Community.chargingStationOuterX - extraSpace,
                                (Community.chargingStationRightY - Community.rightY) / 2.0),
                        new Translation2d(
                                Community.chargingStationOuterX + (longerRobotSide / 2.0) + extraSpace,
                                (Community.chargingStationRightY - Community.rightY) / 2.0)));
        graph.addEdge(
                leftChargingStation,
                noMansLand,
                List.of(
                        new Translation2d(
                                Community.chargingStationOuterX - extraSpace,
                                (Community.chargingStationLeftY - Community.leftY) / 2.0),
                        new Translation2d(
                                Community.chargingStationOuterX + (longerRobotSide / 2.0) + extraSpace,
                                (Community.chargingStationLeftY - Community.leftY) / 2.0)));
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
            System.out.println(i);
            Waypoint current = returnedList.get(i);
            Waypoint next = returnedList.get(i + 1);
            Rectangle currentZone = getZone(current.getTranslation());
            Rectangle nextZone = getZone(next.getTranslation());
            if (currentZone != null && nextZone != null && currentZone != nextZone) {
                List<List<Rectangle>> paths = graph.getPaths(currentZone, nextZone);
                double minDistance = Double.MAX_VALUE;
                List<Translation2d> bestPath = List.of();
                for (List<Rectangle> path : paths) {
                    List<Translation2d> pathPoints = new ArrayList<>();
                    pathPoints.add(current.getTranslation());
                    for (int j = 0; j < path.size() - 1; j++) {
                        pathPoints.addAll(graph.getEdge(path.get(j), path.get(j + 1)));
                    }

                    double distance = 0.0;
                    for (int j = 0; j < pathPoints.size() - 1; j++) {
                        distance += pathPoints.get(j).getDistance(pathPoints.get(j + 1));
                    }
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestPath = pathPoints;
                    }
                }
                // Don't re-add the first and last points
                for (int j = 1; j < bestPath.size(); j++) {
                    returnedList.add(i + j, new Waypoint(bestPath.get(j)));
                }
                System.out.println("SDF" + returnedList.size());
                i += bestPath.size() - 1;
            }
        }
        return returnedList;
    }
}
