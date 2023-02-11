package frc.robot.subsystems.flicker;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlickerSubsystem extends SubsystemBase {

    private boolean FlickerBack = true;

    public enum FlickerState {
        BACK(Value.kReverse),
        FORWARD(Value.kForward);

        public final Value Anothervalue;

        FlickerState(Value Anothervalue) {
            this.Anothervalue = Anothervalue;
        }
    }
    public boolean getState() {
        return FlickerBack;
    }

    private final DoubleSolenoid leftSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_SOLENOID_PORTS[0], LEFT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid rightSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_SOLENOID_PORTS[0], RIGHT_SOLENOID_PORTS[1]);

    public FlickerSubsystem() {}

    public void setFlickerState(FlickerState flickerState) {
        setFlickerState(flickerState.Anothervalue);
        FlickerBack = flickerState == FlickerState.BACK;
    }

    private void setFlickerState(Value Anothervalue) {
        leftSolenoid.set(Anothervalue);
        rightSolenoid.set(Anothervalue);
    }

    public void toggleFlickerState() {
        if (FlickerBack) {
            setFlickerState(FlickerState.FORWARD);
        } else {
            setFlickerState(FlickerState.BACK);
        }
    }
}
