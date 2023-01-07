package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TeleopControlsStateManager {
    enum StateMode {
        IN,
        EXCEPT
    }

    enum State {
        DEFAULT_MODE
    }

    private State currentState = State.DEFAULT_MODE;

    private void setState(State state) {
        currentState = state;
    }

    public CommandBase setStateCommand(State state) {
        return Commands.runOnce(() -> setState(state));
    }

    public Trigger getStatedTrigger(Trigger trigger, StateMode stateMode, State state) {
        switch (stateMode) {
            case IN -> {
                return trigger.and(() -> currentState == state);
            }
            case EXCEPT -> {
                return trigger.and(() -> currentState != state);
            }
        }
        return trigger;
    }
}
