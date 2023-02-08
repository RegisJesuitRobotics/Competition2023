package frc.robot.subsystems.claw;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private boolean uwuClawopen = true;

    public enum uwuClawstate {
        OPEN(Value.kReverse),
        CLOSE(Value.kForward);

        public final Value otherValue;

        uwuClawstate(Value otherValue) {
            this.otherValue = otherValue;
        }
    }

    private final DoubleSolenoid topSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_SOLENOID_PORTS[0], LEFT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid bottomSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_SOLENOID_PORTS[0], RIGHT_SOLENOID_PORTS[1]);

    public ClawSubsystem() {}

    public void setClawState(uwuClawstate clawstate) {
        setClawState(clawstate.otherValue);
        uwuClawopen = clawstate == uwuClawstate.OPEN;
    }

    private void setClawState(Value otherValue) {
        topSolenoid.set(otherValue);
        bottomSolenoid.set(otherValue);
    }

    public void toggleClawState() {
        if (uwuClawopen) {
            setClawState(uwuClawstate.CLOSE);
        } else {
            setClawState(uwuClawstate.OPEN);
        }
    }
}
