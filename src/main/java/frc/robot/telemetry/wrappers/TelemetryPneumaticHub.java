package frc.robot.telemetry.wrappers;

import edu.wpi.first.hal.REVPHFaults;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.SensorUtil;
import frc.robot.telemetry.types.BooleanArrayTelemetryEntry;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.RaiderUtils;

public class TelemetryPneumaticHub extends PneumaticHub {
    private static final String tableName = "/pneumatics/";

    private final DoubleTelemetryEntry inputVoltageEntry;
    private final DoubleTelemetryEntry regulated5VEntry;
    private final DoubleTelemetryEntry totalSolenoidCurrentEntry;
    private final DoubleTelemetryEntry solenoidVoltageEntry;
    private final DoubleTelemetryEntry compressorCurrentEntry;
    private final BooleanTelemetryEntry pressureSwitchEntry;
    private final BooleanTelemetryEntry compressorActiveEntry;
    private final BooleanArrayTelemetryEntry faultsEntry;

    private final Alert faultAlert = new Alert("Pneumatics Hub Fault", AlertType.ERROR);

    private final boolean[] faultArray = new boolean[22];

    public TelemetryPneumaticHub() {
        this(SensorUtil.getDefaultREVPHModule());
    }

    public TelemetryPneumaticHub(int module) {
        super(module);

        String thisTableName = tableName + module + "/";
        inputVoltageEntry = new DoubleTelemetryEntry(thisTableName + "inputVoltage", false);
        regulated5VEntry = new DoubleTelemetryEntry(thisTableName + "regulated5V", false);
        totalSolenoidCurrentEntry = new DoubleTelemetryEntry(thisTableName + "totalSolenoidCurrent", false);
        solenoidVoltageEntry = new DoubleTelemetryEntry(thisTableName + "solenoidVoltage", false);
        compressorCurrentEntry = new DoubleTelemetryEntry(thisTableName + "compressorCurrent", false);
        pressureSwitchEntry = new BooleanTelemetryEntry(thisTableName + "pressureSwitch", false);
        compressorActiveEntry = new BooleanTelemetryEntry(thisTableName + "compressorActive", false);
        faultsEntry = new BooleanArrayTelemetryEntry(thisTableName + "faults", false);
    }

    public void logValues() {
        inputVoltageEntry.append(super.getInputVoltage());
        regulated5VEntry.append(super.get5VRegulatedVoltage());
        totalSolenoidCurrentEntry.append(super.getSolenoidsTotalCurrent());
        solenoidVoltageEntry.append(super.getSolenoidsVoltage());
        compressorCurrentEntry.append(super.getCompressorCurrent());
        pressureSwitchEntry.append(super.getPressureSwitch());
        compressorActiveEntry.append(super.getCompressor());

        revPHFaultsToBooleanArray(super.getFaults(), faultArray);
        faultAlert.set(RaiderUtils.anyTrue(faultArray));
        faultsEntry.append(faultArray);
    }

    private static void revPHFaultsToBooleanArray(REVPHFaults faults, boolean[] toArray) {
        if (toArray.length != 22) {
            throw new IllegalArgumentException("toArray must have length of 22");
        }

        toArray[0] = faults.Channel0Fault;
        toArray[1] = faults.Channel1Fault;
        toArray[2] = faults.Channel2Fault;
        toArray[3] = faults.Channel3Fault;
        toArray[4] = faults.Channel4Fault;
        toArray[5] = faults.Channel5Fault;
        toArray[6] = faults.Channel6Fault;
        toArray[7] = faults.Channel7Fault;
        toArray[8] = faults.Channel8Fault;
        toArray[9] = faults.Channel9Fault;
        toArray[10] = faults.Channel10Fault;
        toArray[11] = faults.Channel11Fault;
        toArray[12] = faults.Channel12Fault;
        toArray[13] = faults.Channel13Fault;
        toArray[14] = faults.Channel14Fault;
        toArray[15] = faults.Channel15Fault;
        toArray[16] = faults.CompressorOverCurrent;
        toArray[17] = faults.CompressorOpen;
        toArray[18] = faults.SolenoidOverCurrent;
        toArray[19] = faults.Brownout;
        toArray[20] = faults.CanWarning;
        toArray[21] = faults.HardwareFault;
    }
}
