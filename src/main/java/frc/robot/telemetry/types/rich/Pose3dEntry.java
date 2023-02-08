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
        copyPose3dToArray(value, pose, 0);

        poseEntry.append(pose);
    }

    @Override
    public void close() {
        poseEntry.close();
    }

    public static void copyPose3dToArray(Pose3d pose, double[] array, int startingIndex) {
        array[startingIndex] = pose.getTranslation().getX();
        array[startingIndex + 1] = pose.getTranslation().getY();
        array[startingIndex + 2] = pose.getTranslation().getZ();
        array[startingIndex + 3] = pose.getRotation().getQuaternion().getW();
        array[startingIndex + 4] = pose.getRotation().getQuaternion().getX();
        array[startingIndex + 5] = pose.getRotation().getQuaternion().getY();
        array[startingIndex + 6] = pose.getRotation().getQuaternion().getZ();
    }
}
