package frc.robot.telemetry.tunable;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunableFFGains {
    public final TunableDouble arbFF;
    public final TunableDouble vFF;
    public final TunableDouble aFF;

    /**
     * @param networkName the name to use for network tables
     * @param arbFF the arbitrary/static feedforward (also known as kS) (in volts per meter/second)
     * @param vFF the velocity feedforward (also known as kV) (in volts per meter/second)
     * @param aFF the acceleration feedforward (also known as kA) (in volts meter/second^2)
     * @param tuningMode if false, the gains will be not be changeable
     */
    public TunableFFGains(String networkName, double arbFF, double vFF, double aFF, boolean tuningMode) {
        networkName += "/";
        this.arbFF = new TunableDouble(networkName + "arbFF", arbFF, tuningMode);
        this.vFF = new TunableDouble(networkName + "vFF", vFF, tuningMode);
        this.aFF = new TunableDouble(networkName + "aFF", aFF, tuningMode);
    }

    public boolean hasChanged() {
        return arbFF.hasChanged() || vFF.hasChanged() || aFF.hasChanged();
    }

    public SimpleMotorFeedforward createFeedforward() {
        return new SimpleMotorFeedforward(arbFF.get(), vFF.get(), aFF.get());
    }
}
