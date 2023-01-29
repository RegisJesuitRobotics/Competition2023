package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class RawTelemetryEntry extends PrimitiveTelemetryEntry {
    private final RawLogEntry logEntry;
    private final RawPublisher networkPublisher;
    private byte[] lastValue;

    public RawTelemetryEntry(String typeString, String path, boolean shouldNT) {
        this(typeString, path, shouldNT, true);
    }

    public RawTelemetryEntry(String typeString, String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new RawLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getRawTopic(path).publish(typeString);
            networkPublisher.setDefault(new byte[0]);
        } else {
            networkPublisher = null;
        }
    }

    public void append(byte[] value) {
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
