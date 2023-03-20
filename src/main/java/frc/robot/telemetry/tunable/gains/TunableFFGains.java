package frc.robot.telemetry.tunable.gains;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunableFFGains {
    public final TunableDouble sFF;
    public final TunableDouble vFF;
    public final TunableDouble aFF;

    /**
     * @param networkName the name to use for network tables
     * @param sFF the arbitrary/static feedforward gain (also known as kS) (in volts)
     * @param vFF the velocity feedforward gain (also known as kV) (in volts per units/second)
     * @param aFF the acceleration feedforward gain (also known as kA) (in volts units/second^2)
     * @param tuningMode if false the gains will be not be changeable
     */
    public TunableFFGains(String networkName, double sFF, double vFF, double aFF, boolean tuningMode) {
        networkName += "/";
        this.sFF = new TunableDouble(networkName + "sFF", sFF, tuningMode);
        this.vFF = new TunableDouble(networkName + "vFF", vFF, tuningMode);
        this.aFF = new TunableDouble(networkName + "aFF", aFF, tuningMode);
    }

    public boolean hasChanged() {
        return sFF.hasChanged() || vFF.hasChanged() || aFF.hasChanged();
    }

    public SimpleMotorFeedforward createFeedforward() {
        return new SimpleMotorFeedforward(sFF.get(), vFF.get(), aFF.get());
    }
}
