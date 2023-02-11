package frc.robot.commands.flicker;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flicker.FlickerSubsystem;
import frc.robot.subsystems.flicker.FlickerSubsystem.FlickerState;

public class FlickerStateCommand extends InstantCommand {
    private final FlickerSubsystem flicker;
    private final FlickerState flickerState;

    public FlickerStateCommand(FlickerState flickerState, FlickerSubsystem flicker) {
        this.flickerState = flickerState;
        
        this.flicker = flicker;
        addRequirements(flicker);
    }

    @Override
    public void initialize() {
        flicker.setFlickerState(flickerState);
    }
}
