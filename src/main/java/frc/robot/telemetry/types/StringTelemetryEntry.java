package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class StringTelemetryEntry extends TelemetryEntry {
    private final StringLogEntry logEntry;
    private final StringPublisher networkPublisher;
    private String lastValue;

    public StringTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public StringTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new StringLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getStringTopic(path).publish();
            networkPublisher.setDefault("");
        } else {
            networkPublisher = null;
        }
    }

    public void append(String value) {
        if (shouldLog(() -> lastValue.equals(value))) {
            logEntry.append(value);

            if (networkPublisher != null) {
                networkPublisher.set(value);
            }
            lastValue = value;
        }
    }
}
