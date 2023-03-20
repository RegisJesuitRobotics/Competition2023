package frc.robot.telemetry.types;

import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class FloatTelemetryEntry extends PrimitiveTelemetryEntry {
    private final FloatLogEntry logEntry;
    private final FloatPublisher networkPublisher;
    private float lastValue;

    public FloatTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public FloatTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new FloatLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getFloatTopic(path).publish();
            networkPublisher.setDefault(0);
        } else {
            networkPublisher = null;
        }
    }

    public void append(float value) {
        if (shouldLog(() -> lastValue == value)) {
            logEntry.append(value);

            if (networkPublisher != null) {
                networkPublisher.set(value);
            }
            lastValue = value;
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
