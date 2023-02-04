package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.TelemetryEntry;

public class SwerveModuleStateArrayEntry implements TelemetryEntry {
    private final DoubleArrayTelemetryEntry logEntry;

    public SwerveModuleStateArrayEntry(String path, boolean shouldNT) {
        logEntry = new DoubleArrayTelemetryEntry(path, shouldNT);
    }

    public void append(SwerveModuleState[] states) {
        double[] values = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            values[i * 2] = states[i].angle.getRadians();
            values[i * 2 + 1] = states[i].speedMetersPerSecond;
        }
        logEntry.append(values);
    }

    @Override
    public void close() {
        logEntry.close();
    }
}
