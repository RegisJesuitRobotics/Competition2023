package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.TelemetryEntry;

public class TrajectoryEntry implements TelemetryEntry {
    private final DoubleArrayTelemetryEntry trajectoryEntry;

    public TrajectoryEntry(String path, boolean shouldNT) {
        trajectoryEntry = new DoubleArrayTelemetryEntry(path, shouldNT);
    }

    /**
     * Appends a trajectory to the entry. Should only be called at the beginning of running a path.
     * @param trajectory the trajectory
     */
    public void append(Trajectory trajectory) {
        double[] value = new double[trajectory.getStates().size() * 3];
        for (int i = 0; i < trajectory.getStates().size(); i++) {
            value[i * 3] = trajectory.getStates().get(i).poseMeters.getX();
            value[i * 3 + 1] = trajectory.getStates().get(i).poseMeters.getY();
            value[i * 3 + 2] = trajectory.getStates().get(i).poseMeters.getRotation().getRadians();
        }

        trajectoryEntry.append(value);
    }

    @Override
    public void close() {
        trajectoryEntry.close();
    }
}
