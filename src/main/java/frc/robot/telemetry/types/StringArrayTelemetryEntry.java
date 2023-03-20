package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class StringArrayTelemetryEntry extends PrimitiveTelemetryEntry {
    private final StringArrayLogEntry logEntry;
    private final StringArrayPublisher networkPublisher;
    private String[] lastValue;

    public StringArrayTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public StringArrayTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new StringArrayLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getStringArrayTopic(path).publish();
            networkPublisher.setDefault(new String[0]);
        } else {
            networkPublisher = null;
        }
    }

    public void append(String[] value) {
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
