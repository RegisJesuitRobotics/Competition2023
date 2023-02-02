package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private boolean intakeDown = true;

    public enum IntakeState {
        DOWN(Value.kReverse),
        UP(Value.kForward);

        public final Value value;

        IntakeState(Value value) {
            this.value = value;
        }
    }

    private final DoubleSolenoid leftSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_SOLENOID_PORTS[0], LEFT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid rightSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_SOLENOID_PORTS[0], RIGHT_SOLENOID_PORTS[1]);

    public IntakeSubsystem() {}

    public void setIntakeState(IntakeState intakeState) {
        setIntakeState(intakeState.value);
        intakeDown = intakeState == IntakeState.DOWN;
    }

    private void setIntakeState(Value value) {
        leftSolenoid.set(value);
        rightSolenoid.set(value);
    }

    public void toggleIntakeState() {
        if (intakeDown) {
            setIntakeState(IntakeState.UP);
        } else {
            setIntakeState(IntakeState.DOWN);
        }
    }
}
