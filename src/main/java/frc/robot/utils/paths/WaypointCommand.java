package frc.robot.utils.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.trajectory.Waypoint;

public class WaypointCommand {
    private Waypoint waypoint;
    private Command command;

    public WaypointCommand(Waypoint waypoint, Command command) {
        this.waypoint = waypoint;
        this.command = command;
    }

    public WaypointCommand(Waypoint waypoint) {
        this.waypoint = waypoint;
        command = null;
    }

    public Command getCommand() {
        return command;
    }

    public Waypoint getWaypoint() {
        return waypoint;
    }
}
