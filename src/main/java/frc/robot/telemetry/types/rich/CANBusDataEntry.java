package frc.robot.telemetry.types.rich;

import edu.wpi.first.hal.can.CANStatus;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;

public class CANBusDataEntry {
    private final DoubleTelemetryEntry percentUtilizationEntry;
    private final IntegerTelemetryEntry offCountEntry;
    private final IntegerTelemetryEntry txErrorCountEntry;
    private final IntegerTelemetryEntry rxErrorCountEntry;
    private final IntegerTelemetryEntry transmitFullCountEntry;

    public CANBusDataEntry(String path, boolean shouldNT) {
        path += "/";
        percentUtilizationEntry = new DoubleTelemetryEntry(path + "percentUtilization", shouldNT);
        offCountEntry = new IntegerTelemetryEntry(path + "offCount", shouldNT);
        txErrorCountEntry = new IntegerTelemetryEntry(path + "transmitErrorCount", shouldNT);
        rxErrorCountEntry = new IntegerTelemetryEntry(path + "receiveErrorCount", shouldNT);
        transmitFullCountEntry = new IntegerTelemetryEntry(path + "transmitFullCount", shouldNT);
    }

    public void append(CANStatus data) {
        percentUtilizationEntry.append(data.percentBusUtilization);
        offCountEntry.append(data.busOffCount);
        txErrorCountEntry.append(data.transmitErrorCount);
        rxErrorCountEntry.append(data.receiveErrorCount);
        transmitFullCountEntry.append(data.txFullCount);
    }
}
