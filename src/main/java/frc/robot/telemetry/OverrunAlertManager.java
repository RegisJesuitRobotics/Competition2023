package frc.robot.telemetry;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;

public class OverrunAlertManager {
    private final LinearFilter loopOverrunFilter = LinearFilter.movingAverage(100);
    private final Alert highLoopOverrunAlert;

    public OverrunAlertManager() {
        highLoopOverrunAlert = new Alert("High frequency of loop overruns", AlertType.WARNING);
    }

    public void update(boolean didLastLoopOverrun) {
        // Over 25% of the last 100 loops were overruns
        highLoopOverrunAlert.set(loopOverrunFilter.calculate(didLastLoopOverrun ? 1.0 : 0.0) > 0.25);
    }
}
