package frc.robot.subsystems.claw;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private boolean clawOpen = true;

    public enum ClawState {
        OPEN(Value.kForward),
        CLOSE(Value.kReverse);

        public final Value value;

        ClawState(Value value) {
            this.value = value;
        }
    }

    private final DoubleSolenoid topSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, TOP_SOLENOID_PORTS[0], TOP_SOLENOID_PORTS[1]);

    private final DoubleSolenoid bottomSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, BOTTOM_SOLENOID_PORTS[0], BOTTOM_SOLENOID_PORTS[1]);

    public ClawSubsystem() {}

    public void setClawState(ClawState clawstate) {
        setClawState(clawstate.value);
        clawOpen = clawstate == ClawState.CLOSE;
    }

    private void setClawState(Value value) {
        topSolenoid.set(value);
        bottomSolenoid.set(value);
    }

    public void toggleClawState() {
        if (clawOpen) {
            setClawState(ClawState.OPEN);
        } else {
            setClawState(ClawState.CLOSE);
        }
    }
}
