package frc.robot.telemetry.types;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class IntegerTelemetryEntry extends TelemetryEntry {
    private final IntegerLogEntry logEntry;
    private final IntegerPublisher networkPublisher;
    private int lastValue;

    public IntegerTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public IntegerTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new IntegerLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getIntegerTopic(path).publish();
            networkPublisher.setDefault(0);
        } else {
            networkPublisher = null;
        }
    }

    public void append(int value) {
        if (shouldLog(() -> lastValue == value)) {
            logEntry.append(value);

            if (networkPublisher != null) {
                networkPublisher.set(value);
            }
            lastValue = value;
        }
    }
}
