package frc.robot.telemetry.wrappers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TelemetryTalonFX extends TalonFX {
    private final DoubleTelemetryEntry outputAmpsEntry;
    private final DoubleTelemetryEntry inputAmpsEntry;
    private final DoubleTelemetryEntry outputPercentEntry;
    private final DoubleTelemetryEntry temperatureEntry;
    private final BooleanTelemetryEntry inBrakeModeEntry;

    public TelemetryTalonFX(int deviceNumber, String telemetryPath, String canbus) {
        super(deviceNumber, canbus);

        telemetryPath += "/";
        outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", true);
        inputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "inputAmps", false);
        outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", true);
        temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", false);
        inBrakeModeEntry = new BooleanTelemetryEntry(telemetryPath + "inBrakeMode", true);
    }

    public TelemetryTalonFX(int deviceNumber, String logTable) {
        this(deviceNumber, logTable, "");
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        inBrakeModeEntry.append(neutralMode == NeutralMode.Brake);
        super.setNeutralMode(neutralMode);
    }

    public void logValues() {
        outputAmpsEntry.append(super.getStatorCurrent());
        inputAmpsEntry.append(super.getSupplyCurrent());
        outputPercentEntry.append(super.getMotorOutputPercent());
        temperatureEntry.append(super.getTemperature());
    }
}
