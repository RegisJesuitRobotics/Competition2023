package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.TelemetryEntry;

public class Pose3dEntry implements TelemetryEntry {
    private final DoubleArrayTelemetryEntry poseEntry;

    public Pose3dEntry(String path, boolean shouldNT) {
        poseEntry = new DoubleArrayTelemetryEntry(path, shouldNT);
    }

    double[] pose = new double[7];

    public void append(Pose3d value) {
        pose[0] = value.getTranslation().getX();
        pose[1] = value.getTranslation().getY();
        pose[2] = value.getTranslation().getZ();
        pose[3] = value.getRotation().getQuaternion().getW();
        pose[4] = value.getRotation().getQuaternion().getX();
        pose[5] = value.getRotation().getQuaternion().getY();
        pose[6] = value.getRotation().getQuaternion().getZ();

        poseEntry.append(pose);
    }

    @Override
    public void close() {
        poseEntry.close();
    }
}
