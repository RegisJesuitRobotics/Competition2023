package frc.robot.commands.flipper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlipperConstants;
import frc.robot.subsystems.intake.FlipperSubsystem;
import frc.robot.subsystems.intake.FlipperSubsystem.InOutState;
import frc.robot.subsystems.intake.FlipperSubsystem.UpDownState;

public class FullyToggleFlipperCommand extends CommandBase {
    private final FlipperSubsystem flipperSubsystem;

    private double startTime;
    private boolean hasDeployedInOut = false;

    public FullyToggleFlipperCommand(FlipperSubsystem flipperSubsystem) {
        this.flipperSubsystem = flipperSubsystem;

        addRequirements(flipperSubsystem);
    }

    @Override
    public void initialize() {
        hasDeployedInOut = false;
        startTime = Timer.getFPGATimestamp();
        flipperSubsystem.setUpDownState(UpDownState.DOWN);
        // Make sure we are in before we go in
        flipperSubsystem.setInOutState(InOutState.IN);
    }

    @Override
    public void execute() {
        if (hasDeployedInOut) {
            return;
        }

        if (Timer.getFPGATimestamp() - startTime > FlipperConstants.UP_DOWN_DOWN_TIME) {
            flipperSubsystem.setInOutState(InOutState.OUT);
            hasDeployedInOut = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        flipperSubsystem.setInOutState(InOutState.IN);
        flipperSubsystem.setUpDownState(UpDownState.UP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
