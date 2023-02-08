package frc.robot.commands.uwuclaw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class EGirlToggleCommand extends InstantCommand {
    private final ClawSubsystem clawSubsystem;

    public EGirlToggleCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize() {
        clawSubsystem.toggleClawState();
    }
}
