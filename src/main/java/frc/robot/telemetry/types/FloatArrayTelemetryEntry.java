package frc.robot.telemetry.types;

import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class FloatArrayTelemetryEntry extends PrimitiveTelemetryEntry {
    private final FloatArrayLogEntry logEntry;
    private final FloatArrayPublisher networkPublisher;
    private float[] lastValue;

    public FloatArrayTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public FloatArrayTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new FloatArrayLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getFloatArrayTopic(path).publish();
            networkPublisher.setDefault(new float[0]);
        } else {
            networkPublisher = null;
        }
    }

    public void append(float[] value) {
        if (shouldLog(() -> Arrays.equals(lastValue, value))) {
            logEntry.append(value);

            if (networkPublisher != null) {
                networkPublisher.set(value);
            }
            lastValue = Arrays.copyOf(value, value.length);
        }
    }

    @Override
    public void close() {
        logEntry.finish();
        if (networkPublisher != null) {
            networkPublisher.close();
        }
    }
}
