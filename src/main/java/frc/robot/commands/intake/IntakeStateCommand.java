package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class IntakeStateCommand extends InstantCommand {
    private final IntakeSubsystem intake;
    private final IntakeState intakeState;

    public IntakeStateCommand(IntakeState intakeState, IntakeSubsystem intake) {
        this.intakeState = intakeState;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setIntakeState(intakeState);
    }
}
