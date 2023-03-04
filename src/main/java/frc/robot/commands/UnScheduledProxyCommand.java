package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class UnScheduledProxyCommand extends CommandBase {
    private final Command command;

    public UnScheduledProxyCommand(Command command) {
        this.command = command;

        addRequirements(command.getRequirements().toArray(new Subsystem[0]));
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }
}
