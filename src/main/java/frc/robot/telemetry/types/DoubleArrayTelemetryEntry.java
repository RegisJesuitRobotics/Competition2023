package frc.robot.telemetry.types;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class DoubleArrayTelemetryEntry extends TelemetryEntry {
    private final DoubleArrayLogEntry logEntry;
    private final DoubleArrayPublisher networkPublisher;
    private double[] lastValue;

    public DoubleArrayTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public DoubleArrayTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getDoubleArrayTopic(path).publish();
            networkPublisher.setDefault(new double[0]);
        } else {
            networkPublisher = null;
        }
    }

    public void append(double[] value) {
        if (shouldLog(() -> Arrays.equals(lastValue, value))) {
            logEntry.append(value);

            if (networkPublisher != null) {
                networkPublisher.set(value);
            }
            lastValue = Arrays.copyOf(value, value.length);
        }
    }
}
