package frc.robot.utils.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class AutoAction {
    public enum PositionNeed {
        NONE,
        INSIDE_COMMUNITY,
        OUTSIDE_COMMUNITY
    }

    private final List<Waypoint> waypoints;
    private final AutoActionCommands commands;
    private final PositionNeed positionNeed;

    public AutoAction(List<Waypoint> waypoints, AutoActionCommands commands, PositionNeed positionNeed) {
        this.waypoints = waypoints;
        this.commands = commands;
        this.positionNeed = positionNeed;
    }

    public AutoAction(List<Waypoint> waypoints, AutoActionCommands commands) {
        this(waypoints, commands, PositionNeed.NONE);
    }

    public AutoAction(List<Waypoint> waypoints, PositionNeed positionNeed) {
        this(waypoints, new AutoActionCommands.Builder().build(), positionNeed);
    }

    public AutoAction(Waypoint waypoint, AutoActionCommands commands) {
        this(List.of(waypoint), commands);
    }

    public AutoAction(List<Waypoint> waypoints) {
        this(waypoints, new AutoActionCommands.Builder().build(), PositionNeed.NONE);
    }

    public AutoAction(Waypoint waypoint) {
        this(List.of(waypoint));
    }

    public AutoActionCommands getCommands() {
        return commands;
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public PositionNeed getNeed() {
        return positionNeed;
    }

    public AutoAction reversed() {
        List<Waypoint> newWaypointsList = new ArrayList<>(waypoints);
        Collections.reverse(newWaypointsList);
        return new AutoAction(newWaypointsList, commands);
    }

    @Override
    public String toString() {
        return "WaypointsCommandPair{" + "waypoints=" + waypoints + ", commandGenerator=" + commands + ", waypointNeed="
                + positionNeed + "}";
    }

    public static class AutoActionCommands {
        private final Supplier<Command> stoppedCommandGenerator;
        private final Supplier<Command> whileDrivingCommandWithWaitGenerator;
        private final Supplier<Command> whileDrivingCommandNoWaitGenerator;

        private AutoActionCommands(
                Supplier<Command> stoppedCommandGenerator,
                Supplier<Command> whileDrivingCommandWithWaitGenerator,
                Supplier<Command> whileDrivingCommandNoWaitGenerator) {
            this.stoppedCommandGenerator = stoppedCommandGenerator;
            this.whileDrivingCommandWithWaitGenerator = whileDrivingCommandWithWaitGenerator;
            this.whileDrivingCommandNoWaitGenerator = whileDrivingCommandNoWaitGenerator;
        }

        private AutoActionCommands(Supplier<Command> stoppedCommandGenerator) {
            this(stoppedCommandGenerator, null, null);
        }

        public Command getStoppedCommand() {
            if (stoppedCommandGenerator == null) {
                return null;
            }
            return stoppedCommandGenerator.get();
        }

        public Command getWhileDrivingCommand() {
            if (whileDrivingCommandWithWaitGenerator == null) {
                return null;
            }
            return whileDrivingCommandWithWaitGenerator.get();
        }

        public Command getWhileDrivingNoWait() {
            if (whileDrivingCommandNoWaitGenerator == null) {
                return null;
            }
            return whileDrivingCommandNoWaitGenerator.get();
        }

        public static class Builder {
            private Supplier<Command> stoppedCommandGenerator;
            private Supplier<Command> whileDrivingCommandWithWaitGenerator;
            private Supplier<Command> whileDrivingCommandNoWaitGenerator;

            public Builder withStoppedCommand(Supplier<Command> stoppedCommandGenerator) {
                this.stoppedCommandGenerator = stoppedCommandGenerator;
                return this;
            }

            public Builder withWhileDrivingCommandWithWait(Supplier<Command> unStoppedCommandWithWaitGenerator) {
                this.whileDrivingCommandWithWaitGenerator = unStoppedCommandWithWaitGenerator;
                return this;
            }

            public Builder withWhileDrivingCommandNoWait(Supplier<Command> unStoppedCommandNoWaitGenerator) {
                this.whileDrivingCommandNoWaitGenerator = unStoppedCommandNoWaitGenerator;
                return this;
            }

            public AutoActionCommands build() {
                return new AutoActionCommands(
                        stoppedCommandGenerator,
                        whileDrivingCommandWithWaitGenerator,
                        whileDrivingCommandNoWaitGenerator);
            }
        }
    }
}
