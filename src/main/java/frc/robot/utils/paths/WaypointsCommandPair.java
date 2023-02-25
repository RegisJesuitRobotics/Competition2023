package frc.robot.utils.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class WaypointsCommandPair {
    private final List<Waypoint> waypoints;
    private final Command command;

    public WaypointsCommandPair(List<Waypoint> waypoints, Command command) {
        this.waypoints = waypoints;
        this.command = command;
    }

    public WaypointsCommandPair(Waypoint waypoint, Command command) {
        this(List.of(waypoint), command);
    }

    public WaypointsCommandPair(List<Waypoint> waypoints) {
        this(waypoints, null);
    }

    public WaypointsCommandPair(Waypoint waypoint) {
        this(List.of(waypoint));
    }

    public Command getCommand() {
        return command;
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public WaypointsCommandPair reversed() {
        List<Waypoint> newWaypointsList = new ArrayList<>(waypoints);
        Collections.reverse(newWaypointsList);
        return new WaypointsCommandPair(newWaypointsList, command);
    }
}
