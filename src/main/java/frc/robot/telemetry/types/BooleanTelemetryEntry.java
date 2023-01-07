package frc.robot.telemetry.types;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class BooleanTelemetryEntry extends TelemetryEntry {
    private final BooleanLogEntry logEntry;
    private final BooleanPublisher networkPublisher;
    private boolean lastValue;

    public BooleanTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public BooleanTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new BooleanLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getBooleanTopic(path).publish();
            networkPublisher.setDefault(false);
        } else {
            networkPublisher = null;
        }
    }

    public void append(boolean value) {
        if (shouldLog(() -> lastValue == value)) {
            logEntry.append(value);

            if (networkPublisher != null) {
                networkPublisher.set(value);
            }
            lastValue = value;
        }
    }
}
