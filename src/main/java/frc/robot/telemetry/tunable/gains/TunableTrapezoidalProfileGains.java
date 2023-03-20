package frc.robot.telemetry.tunable.gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableTrapezoidalProfileGains {
    public final TunableDouble maxVelocity;
    public final TunableDouble maxAcceleration;

    /**
     * @param networkName the name to use for network tables
     * @param maxVelocity the maximum velocity
     * @param maxAcceleration the maximum acceleration
     * @param tuningMode if false, the gains will be not be changeable
     */
    public TunableTrapezoidalProfileGains(
            String networkName, double maxVelocity, double maxAcceleration, boolean tuningMode) {
        networkName += "/";
        this.maxVelocity = new TunableDouble(networkName + "maxVelocity", maxVelocity, tuningMode);
        this.maxAcceleration = new TunableDouble(networkName + "maxAcceleration", maxAcceleration, tuningMode);
    }

    public boolean hasChanged() {
        return maxVelocity.hasChanged() || maxAcceleration.hasChanged();
    }

    public TrapezoidProfile.Constraints createConstraints() {
        return new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
    }
}
