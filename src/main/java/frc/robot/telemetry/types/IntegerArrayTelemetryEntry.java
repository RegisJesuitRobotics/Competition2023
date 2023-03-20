package frc.robot.telemetry.types;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class IntegerArrayTelemetryEntry extends PrimitiveTelemetryEntry {
    private final IntegerArrayLogEntry logEntry;
    private final IntegerArrayPublisher networkPublisher;
    private long[] lastValue;

    public IntegerArrayTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public IntegerArrayTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new IntegerArrayLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getIntegerArrayTopic(path).publish();
            networkPublisher.setDefault(new long[0]);
        } else {
            networkPublisher = null;
        }
    }

    public void append(long[] value) {
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
