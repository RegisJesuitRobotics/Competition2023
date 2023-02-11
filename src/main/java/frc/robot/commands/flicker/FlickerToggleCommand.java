package frc.robot.commands.flicker;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flicker.FlickerSubsystem;

public class FlickerToggleCommand extends InstantCommand {
    private final FlickerSubsystem flicker;

    public FlickerToggleCommand(FlickerSubsystem flicker) {
        this.flicker = flicker;
    }

    @Override
    public void initialize() {
        flicker.toggleFlickerState();
    }
}
