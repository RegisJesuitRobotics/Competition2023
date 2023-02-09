package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawToggleCommand extends InstantCommand {
    private final ClawSubsystem clawSubsystem;

    public ClawToggleCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize() {
        clawSubsystem.toggleClawState();
    }
}
