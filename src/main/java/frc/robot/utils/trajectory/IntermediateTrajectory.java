package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.paths.WaypointCommand;
import java.util.ArrayList;
import java.util.List;

public class IntermediateTrajectory {
    private List<WaypointCommand> waypointCommands;

    private boolean generateNewCommand;
    private Command command;

    public IntermediateTrajectory(List<WaypointCommand> waypoints, boolean generateNewPath) {
        this.waypointCommands = waypoints;
        this.command = waypoints.get(1).getCommand();
        this.generateNewCommand = generateNewPath;
    }

    public List<Waypoint> getWaypoint() {
        List<Waypoint> waypoints = new ArrayList<>();
        for (WaypointCommand waypointCommand : waypointCommands) {
            waypoints.add(waypointCommand.getWaypoint());
        }
        return waypoints;
    }

    public Command getCommand() {
        return command;
    }
}
