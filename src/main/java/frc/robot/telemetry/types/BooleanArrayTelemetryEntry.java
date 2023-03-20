package frc.robot.telemetry.types;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class BooleanArrayTelemetryEntry extends PrimitiveTelemetryEntry {
    private final BooleanArrayLogEntry logEntry;
    private final BooleanArrayPublisher networkPublisher;
    private boolean[] lastValue;

    public BooleanArrayTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public BooleanArrayTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new BooleanArrayLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getBooleanArrayTopic(path).publish();
            networkPublisher.setDefault(new boolean[0]);
        } else {
            networkPublisher = null;
        }
    }

    public void append(boolean[] value) {
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
