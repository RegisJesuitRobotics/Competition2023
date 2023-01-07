package frc.robot.telemetry.tunable;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TunableTelemetryPIDController extends PIDController {
    private final DoubleTelemetryEntry currentMeasurementEntry;
    private final DoubleTelemetryEntry setpointEntry;
    private final DoubleTelemetryEntry outputEntry;
    private final TunablePIDGains gains;

    public TunableTelemetryPIDController(String logTable, TunablePIDGains gains) {
        this(logTable, gains, 0.02);
    }

    public TunableTelemetryPIDController(String logTable, TunablePIDGains gains, double period) {
        super(gains.p.get(), gains.i.get(), gains.d.get(), period);
        this.gains = gains;

        logTable += "/";
        currentMeasurementEntry = new DoubleTelemetryEntry(logTable + "currentMeasurement", true);
        setpointEntry = new DoubleTelemetryEntry(logTable + "setpoint", true);
        outputEntry = new DoubleTelemetryEntry(logTable + "output", true);
    }

    @Override
    public double calculate(double measurement) {
        if (gains.hasChanged()) {
            setPID(gains.p.get(), gains.i.get(), gains.d.get());
        }

        currentMeasurementEntry.append(measurement);
        setpointEntry.append(getSetpoint());

        double output = super.calculate(measurement);
        outputEntry.append(output);

        return output;
    }
}
