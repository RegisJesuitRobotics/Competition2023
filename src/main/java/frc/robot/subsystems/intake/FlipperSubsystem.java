package frc.robot.subsystems.intake;

import static frc.robot.Constants.FlipperConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.utils.RaiderUtils;

public class FlipperSubsystem extends SubsystemBase {
    public enum UpDownState {
        UP(Value.kReverse),
        DOWN(Value.kForward);

        public final Value value;

        UpDownState(Value value) {
            this.value = value;
        }
    }

    public enum InOutState {
        IN(Value.kReverse),
        OUT(Value.kForward);

        public final Value value;

        InOutState(Value value) {
            this.value = value;
        }
    }

    private final DoubleSolenoid upDownSolenoid = new DoubleSolenoid(
            MiscConstants.PNEUMATICS_MODULE_TYPE, UP_DOWN_SOLENOID_PORTS[0], UP_DOWN_SOLENOID_PORTS[1]);
    private final DoubleSolenoid inOutSolenoid = new DoubleSolenoid(
            MiscConstants.PNEUMATICS_MODULE_TYPE, LEFT_RIGHT_SOLENOID_PORTS[0], LEFT_RIGHT_SOLENOID_PORTS[1]);

    private final IntegerTelemetryEntry upDownSolenoidEntry =
            new IntegerTelemetryEntry("/flipper/upDownSolenoid", false);
    private final IntegerTelemetryEntry inOutSolenoidEntry = new IntegerTelemetryEntry("/flipper/inOutSolenoid", false);

    public FlipperSubsystem() {}

    public void setUpDownState(UpDownState upDownState) {
        upDownSolenoid.set(upDownState.value);
    }

    public void toggleUpDownState() {
        // Even if it's off, go up
        if (upDownSolenoid.get() == UpDownState.UP.value) {
            setUpDownState(UpDownState.DOWN);
        } else {
            setUpDownState(UpDownState.UP);
        }
    }

    public void setInOutState(InOutState inOutState) {
        inOutSolenoid.set(inOutState.value);
    }

    public void toggleInOutState() {
        // Even if it's off, go in
        if (inOutSolenoid.get() == InOutState.OUT.value) {
            setInOutState(InOutState.IN);
        } else {
            setInOutState(InOutState.OUT);
        }
    }

    @Override
    public void periodic() {
        Robot.startWNode("FlipperSubsystem");
        Robot.startWNode("logValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        upDownSolenoidEntry.append(RaiderUtils.getSolenoidValueToInt(upDownSolenoid.get()));
        inOutSolenoidEntry.append(RaiderUtils.getSolenoidValueToInt(inOutSolenoid.get()));
    }
}
