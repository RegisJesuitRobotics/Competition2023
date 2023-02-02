package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeToggleCommand extends InstantCommand {
    private final IntakeSubsystem intake;

    public IntakeToggleCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.toggleIntakeState();
    }
}
