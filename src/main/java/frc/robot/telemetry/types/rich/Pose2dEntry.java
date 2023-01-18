package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.TelemetryEntry;

public class Pose2dEntry implements TelemetryEntry {
    private final DoubleTelemetryEntry xEntry;
    private final DoubleTelemetryEntry yEntry;
    private final DoubleTelemetryEntry rotationEntry;

    public Pose2dEntry(String path, boolean shouldNT) {
        path += "/";
        xEntry = new DoubleTelemetryEntry(path + "x", shouldNT);
        yEntry = new DoubleTelemetryEntry(path + "y", shouldNT);
        rotationEntry = new DoubleTelemetryEntry(path + "rotation", shouldNT);
    }

    public void append(Pose2d value) {
        xEntry.append(value.getX());
        yEntry.append(value.getY());
        rotationEntry.append(value.getRotation().getRadians());
    }

    @Override
    public void close() {
        xEntry.close();
        yEntry.close();
        rotationEntry.close();
    }
}
