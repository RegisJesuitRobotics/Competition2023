package frc.robot.telemetry.types;

import java.util.function.BooleanSupplier;

public abstract class PrimitiveTelemetryEntry implements TelemetryEntry {
    protected final boolean shouldLazyLog;

    protected boolean firstRun = true;

    protected PrimitiveTelemetryEntry(boolean shouldLazyLog) {
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
