package frc.robot.telemetry.types;

import java.util.function.BooleanSupplier;

public abstract class TelemetryEntry {
    protected final boolean shouldLazyLog;

    protected boolean firstRun = true;

    protected TelemetryEntry(boolean shouldLazyLog) {
        this.shouldLazyLog = shouldLazyLog;
    }

    protected boolean shouldLog(BooleanSupplier isLastEqual) {
        if (firstRun) {
            firstRun = false;
            return true;
        }
        return !shouldLazyLog || !isLastEqual.getAsBoolean();
    }
}
