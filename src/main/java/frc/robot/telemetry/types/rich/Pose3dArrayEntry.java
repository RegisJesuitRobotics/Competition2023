package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.TelemetryEntry;
import java.util.List;

public class Pose3dArrayEntry implements TelemetryEntry {
    private final DoubleArrayTelemetryEntry posesEntry;

    public Pose3dArrayEntry(String path, boolean shouldNT) {
        posesEntry = new DoubleArrayTelemetryEntry(path, shouldNT);
    }

    public void append(Pose3d... values) {
        double[] poses = new double[values.length * 7];
        for (int i = 0; i < values.length; i++) {
            Pose3dEntry.copyPose3dToArray(values[i], poses, i * 7);
        }

        posesEntry.append(poses);
    }

    public void append(List<Pose3d> values) {
        double[] poses = new double[values.size() * 7];
        for (int i = 0; i < values.size(); i++) {
            Pose3dEntry.copyPose3dToArray(values.get(i), poses, i * 7);
        }

        posesEntry.append(poses);
    }

    @Override
    public void close() {
        posesEntry.close();
    }
}
