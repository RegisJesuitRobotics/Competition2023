package frc.robot.subsystems.claw;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.utils.RaiderUtils;

public class ClawSubsystem extends SubsystemBase {
    public enum ClawState {
        OPEN(Value.kReverse),
        CLOSE(Value.kForward);

        public final Value value;

        ClawState(Value value) {
            this.value = value;
        }
    }

    private final DoubleSolenoid solenoid =
            new DoubleSolenoid(MiscConstants.PNEUMATICS_MODULE_TYPE, SOLENOID_PORTS[0], SOLENOID_PORTS[1]);

    private final BooleanTelemetryEntry isOpenEntry = new BooleanTelemetryEntry("/claw/isOpen", true);
    private final IntegerTelemetryEntry solenoidEntry = new IntegerTelemetryEntry("/claw/solenoid", false);

    public ClawSubsystem() {}

    public void setClawState(ClawState clawstate) {
        solenoid.set(clawstate.value);
    }

    public void toggleClawState() {
        // Even if it's off, go open
        if (solenoid.get() == ClawState.CLOSE.value) {
            setClawState(ClawState.OPEN);
        } else {
            setClawState(ClawState.CLOSE);
        }
    }

    @Override
    public void periodic() {
        Robot.startWNode("ClawSubsystem#periodic");
        Robot.startWNode("logValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        solenoidEntry.append(RaiderUtils.getSolenoidValueToInt(solenoid.get()));
        isOpenEntry.append(solenoid.get() == ClawState.OPEN.value);
    }
}
