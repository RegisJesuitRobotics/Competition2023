package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawState;

public class ClawStateCommand extends InstantCommand {
    private final ClawSubsystem clawSubsystem;
    private final ClawState clawState;

    public ClawStateCommand(ClawState clawState, ClawSubsystem clawSubsystem) {
        this.clawState = clawState;
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawState(clawState);
    }
}
