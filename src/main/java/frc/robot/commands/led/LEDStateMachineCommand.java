package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.utils.led.Pattern;
import java.util.List;
import java.util.function.BooleanSupplier;

public class LEDStateMachineCommand extends CommandBase {
    private final Pattern defaultPattern;
    private final List<LEDState> states;
    private final LEDSubsystem ledSubsystem;

    public LEDStateMachineCommand(Pattern defaultPattern, List<LEDState> states, LEDSubsystem ledSubsystem) {
        this.defaultPattern = defaultPattern;

        this.states = states;
        this.ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        LEDState mostPriorityTrueState = null;
        for (LEDState state : states) {
            if (state.condition.getAsBoolean()) {
                mostPriorityTrueState = state;
                break;
            }
        }

        Pattern selectedPattern = defaultPattern;
        if (mostPriorityTrueState != null) {
            selectedPattern = mostPriorityTrueState.pattern;
        }
        ledSubsystem.setAllPattern(selectedPattern);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    /**
     * @param condition the condition to be true for this state to be active
     * @param pattern the pattern to be used when this state is active
     */
    public record LEDState(BooleanSupplier condition, Pattern pattern) {}
}
