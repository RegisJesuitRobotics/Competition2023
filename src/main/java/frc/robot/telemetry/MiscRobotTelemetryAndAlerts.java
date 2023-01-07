package frc.robot.telemetry;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.rich.CANBusDataEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class MiscRobotTelemetryAndAlerts {
    private static final String tableName = "/robot/";

    private final Alert lowBatteryVoltageAlert = new Alert("Low Battery Voltage", AlertType.WARNING);
    private final Alert highCanUsageAlert = new Alert("High CAN Usage", AlertType.WARNING);
    private final LinearFilter highCanUsageFilter = LinearFilter.movingAverage(50);

    private final Alert[] controllerAlerts = new Alert[MiscConstants.USED_CONTROLLER_PORTS.length];

    private final DoubleTelemetryEntry voltageEntry = new DoubleTelemetryEntry(tableName + "voltage", false);
    private final CANBusDataEntry canBusDataEntry = new CANBusDataEntry(tableName + "can", false);

    public MiscRobotTelemetryAndAlerts() {
        for (int i = 0; i < controllerAlerts.length; i++) {
            controllerAlerts[i] = new Alert(
                    "Controller " + MiscConstants.USED_CONTROLLER_PORTS[i] + " is disconnected.", AlertType.WARNING);
        }

        if (MiscConstants.TUNING_MODE) {
            Alert tuningModeAlert = new Alert("Tuning Mode is Enabled", AlertType.INFO);
            tuningModeAlert.set(true);
        }

        loadAndSetBuildTimeAlert();
    }

    private void loadAndSetBuildTimeAlert() {
        File buildTimeFile = new File(Filesystem.getDeployDirectory(), "buildTime.txt");
        Alert buildTimeAlert = null;
        try (FileReader buildTimeReader = new FileReader(buildTimeFile)) {
            char[] date = new char[19];
            int read = buildTimeReader.read(date);
            if (read == 19) {
                buildTimeAlert = new Alert("Robot code was built " + new String(date) + ".", AlertType.INFO);
            }
        } catch (IOException ignored) {
        }

        if (buildTimeAlert == null) {
            buildTimeAlert = new Alert("Build time file could not be read.", AlertType.WARNING);
        }

        buildTimeAlert.set(true);
    }

    public void logValues() {
        // Battery voltage
        double batteryVoltage = RobotController.getBatteryVoltage();
        voltageEntry.append(batteryVoltage);
        lowBatteryVoltageAlert.set(batteryVoltage < 11.0);

        CANStatus canStatus = RobotController.getCANStatus();
        canBusDataEntry.append(canStatus);

        // CAN Usage
        double percentBusUsage = canStatus.percentBusUtilization;
        double filtered = highCanUsageFilter.calculate(percentBusUsage);
        highCanUsageAlert.set(filtered >= 0.9);

        // Joysticks
        for (int i = 0; i < controllerAlerts.length; i++) {
            controllerAlerts[i].set(!DriverStation.isJoystickConnected(MiscConstants.USED_CONTROLLER_PORTS[i]));
        }
    }
}
