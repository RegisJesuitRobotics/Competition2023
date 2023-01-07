package frc.robot.telemetry.tunable;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

public class TunablePIDGains {
    public final TunableDouble p;
    public final TunableDouble i;
    public final TunableDouble d;

    /**
     * @param networkName the name to use for network tables
     * @param p the p gain
     * @param i the i gain
     * @param d the d gain
     * @param tuningMode if false, the gains will be not be changeable
     */
    public TunablePIDGains(String networkName, double p, double i, double d, boolean tuningMode) {
        networkName += "/";
        this.p = new TunableDouble(networkName + "p", p, tuningMode);
        this.i = new TunableDouble(networkName + "i", i, tuningMode);
        this.d = new TunableDouble(networkName + "d", d, tuningMode);
    }

    public void setSlot(SlotConfiguration slot) {
        slot.kP = p.get();
        slot.kI = i.get();
        slot.kD = d.get();
    }

    public boolean hasChanged() {
        return p.hasChanged() || i.hasChanged() || d.hasChanged();
    }

    public TunableTelemetryPIDController createLoggablePIDController(String logTable) {
        return new TunableTelemetryPIDController(logTable, this);
    }
}
