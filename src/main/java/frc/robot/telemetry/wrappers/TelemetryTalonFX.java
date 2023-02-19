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
    private final DoubleTelemetryEntry positionEntry;
    private final DoubleTelemetryEntry velocityEntry;

    private double loggingPositionConversionFactor = 1.0;
    private double loggingVelocityConversionFactor = 1.0;

    public TelemetryTalonFX(int deviceNumber, String telemetryPath, String canbus, boolean tuningMode) {
        super(deviceNumber, canbus);

        telemetryPath += "/";
        outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", tuningMode);
        inputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "inputAmps", tuningMode);
        outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", tuningMode);
        temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", tuningMode);
        inBrakeModeEntry = new BooleanTelemetryEntry(telemetryPath + "inBrakeMode", tuningMode);
        positionEntry = new DoubleTelemetryEntry(telemetryPath + "position", tuningMode);
        velocityEntry = new DoubleTelemetryEntry(telemetryPath + "velocity", tuningMode);
    }

    public TelemetryTalonFX(int deviceNumber, String logTable, boolean tuningMode) {
        this(deviceNumber, logTable, "", tuningMode);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        inBrakeModeEntry.append(neutralMode == NeutralMode.Brake);
        super.setNeutralMode(neutralMode);
    }

    public void setLoggingPositionConversionFactor(double loggingPositionConversionFactor) {
        this.loggingPositionConversionFactor = loggingPositionConversionFactor;
    }

    public void setLoggingVelocityConversionFactor(double loggingVelocityConversionFactor) {
        this.loggingVelocityConversionFactor = loggingVelocityConversionFactor;
    }

    public void logValues() {
        outputAmpsEntry.append(super.getStatorCurrent());
        inputAmpsEntry.append(super.getSupplyCurrent());
        outputPercentEntry.append(super.getMotorOutputPercent());
        temperatureEntry.append(super.getTemperature());
        positionEntry.append(super.getSelectedSensorPosition() * loggingPositionConversionFactor);
        velocityEntry.append(super.getSelectedSensorVelocity() * loggingVelocityConversionFactor);
    }
}
