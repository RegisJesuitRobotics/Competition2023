package frc.robot.commands.extension;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.extension.ExtensionSubsystem;

public class SetExtensionPositionCommand extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;
    private final double desiredPosition;

    public SetExtensionPositionCommand(double desiredPosition, ExtensionSubsystem extensionSubsystem) {
        this.desiredPosition = desiredPosition;
        this.extensionSubsystem = extensionSubsystem;

        addRequirements(extensionSubsystem);
    }

    @Override
    public void initialize() {
        extensionSubsystem.setDesiredPosition(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        extensionSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return extensionSubsystem.atClosedLoopGoal();
    }
}
