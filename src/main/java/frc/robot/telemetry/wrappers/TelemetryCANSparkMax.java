package frc.robot.telemetry.wrappers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.utils.SparkMaxFlashManager;

public class TelemetryCANSparkMax extends CANSparkMax {
    private final DoubleTelemetryEntry outputAmpsEntry;
    private final DoubleTelemetryEntry outputPercentEntry;
    private final DoubleTelemetryEntry temperatureEntry;
    private final BooleanTelemetryEntry inBrakeModeEntry;

    public TelemetryCANSparkMax(int deviceId, MotorType type, String telemetryPath) {
        super(deviceId, type);

        telemetryPath += "/";

        outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", true);
        outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", true);
        temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", false);
        inBrakeModeEntry = new BooleanTelemetryEntry(telemetryPath + "inBrakeMode", true);

        IntegerTelemetryEntry firmwareVersionEntry =
                new IntegerTelemetryEntry(telemetryPath + "firmwareVersion", false);
        firmwareVersionEntry.append(super.getFirmwareVersion());
        firmwareVersionEntry.close();
    }

    @Override
    public REVLibError setIdleMode(IdleMode mode) {
        inBrakeModeEntry.append(mode == IdleMode.kBrake);

        return super.setIdleMode(mode);
    }

    public void logValues() {
        outputAmpsEntry.append(super.getOutputCurrent());
        outputPercentEntry.append(super.getAppliedOutput());
        temperatureEntry.append(super.getMotorTemperature());
    }

    /**
     * Burns the flash if it should according to {@link frc.robot.utils.SparkMaxFlashManager}
     */
    public REVLibError burnFlashIfShould() {
        if (SparkMaxFlashManager.shouldFlash()) {
            return super.burnFlash();
        }
        return REVLibError.kOk;
    }
}
