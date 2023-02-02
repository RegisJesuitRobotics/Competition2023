package frc.robot.telemetry.tunable;
// Modified from 6328 Mechanical Advantage

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableDouble {
    private final DoubleEntry networkEntry;
    private final DoubleTelemetryEntry telemetryEntry;
    private final boolean tuningMode;
    private final double defaultValue;
    private double lastHasChangedValue;

    /**
     * Create a new TunableNumber with the default value
     *
     * @param networkName Name for network tables
     * @param defaultValue Default value
     * @param tuningMode If true, will get value from dashboard
     */
    public TunableDouble(String networkName, double defaultValue, boolean tuningMode) {
        this.networkEntry =
                NetworkTableInstance.getDefault().getDoubleTopic(networkName).getEntry(defaultValue);
        // Make sure it gets reset on each deploy
        networkEntry.set(defaultValue);
        this.telemetryEntry = new DoubleTelemetryEntry(networkName, false);
        this.defaultValue = defaultValue;
        this.tuningMode = tuningMode;

        this.lastHasChangedValue = defaultValue;
    }

    /**
     * Get the current value from NT if available
     *
     * @return The current value
     */
    public double get() {
        if (!tuningMode) {
            return defaultValue;
        }
        double value = networkEntry.get();
        telemetryEntry.append(value);
        return value;
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }

        return false;
    }
}
