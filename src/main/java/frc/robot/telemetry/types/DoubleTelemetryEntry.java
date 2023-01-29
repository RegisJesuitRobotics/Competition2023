package frc.robot.telemetry.types;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class DoubleTelemetryEntry extends PrimitiveTelemetryEntry {
    private final DoubleLogEntry logEntry;
    private final DoublePublisher networkPublisher;
    private double lastValue;

    public DoubleTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public DoubleTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(shouldLazyLog);

        logEntry = new DoubleLogEntry(DataLogManager.getLog(), path);
        if (shouldNT) {
            networkPublisher =
                    NetworkTableInstance.getDefault().getDoubleTopic(path).publish();
            networkPublisher.setDefault(0.0);
        } else {
            networkPublisher = null;
        }
    }

    public void append(double value) {
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
