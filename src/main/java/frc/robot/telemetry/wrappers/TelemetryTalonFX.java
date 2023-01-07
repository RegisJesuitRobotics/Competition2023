package frc.robot.telemetry.wrappers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;

public class TelemetryTalonFX extends TalonFX {
    private final DoubleTelemetryEntry outputAmpsEntry;
    private final DoubleTelemetryEntry inputAmpsEntry;
    private final DoubleTelemetryEntry outputPercentEntry;
    private final DoubleTelemetryEntry temperatureEntry;
    private final BooleanTelemetryEntry inBrakeModeEntry;

    public TelemetryTalonFX(int deviceNumber, String logTable, String canbus) {
        super(deviceNumber, canbus);

        logTable += "/";
        outputAmpsEntry = new DoubleTelemetryEntry(logTable + "outputAmps", true);
        inputAmpsEntry = new DoubleTelemetryEntry(logTable + "inputAmps", false);
        outputPercentEntry = new DoubleTelemetryEntry(logTable + "outputPercent", true);
        temperatureEntry = new DoubleTelemetryEntry(logTable + "temperature", false);
        inBrakeModeEntry = new BooleanTelemetryEntry(logTable + "inBrakeMode", true);

        IntegerTelemetryEntry firmwareVersionEntry = new IntegerTelemetryEntry(logTable + "firmwareVersion", false);
        firmwareVersionEntry.append(super.getFirmwareVersion());
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
