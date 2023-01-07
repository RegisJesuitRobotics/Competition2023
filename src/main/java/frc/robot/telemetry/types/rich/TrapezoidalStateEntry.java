package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TrapezoidalStateEntry {
    private final DoubleTelemetryEntry positionEntry;
    private final DoubleTelemetryEntry velocityEntry;

    public TrapezoidalStateEntry(String path, boolean shouldNT) {
        path += "/";
        positionEntry = new DoubleTelemetryEntry(path + "position", shouldNT);
        velocityEntry = new DoubleTelemetryEntry(path + "velocity", shouldNT);
    }

    public void append(TrapezoidProfile.State state) {
        positionEntry.append(state.position);
        velocityEntry.append(state.velocity);
    }
}
