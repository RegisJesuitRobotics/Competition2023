package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class CorrectBalanceAndLockCommand extends SequentialCommandGroup {
    public CorrectBalanceAndLockCommand(SwerveDriveSubsystem driveSubsystem) {
        addCommands(
                //                new CorrectBalancePart1Command(driveSubsystem),
                new LockModulesCommand(driveSubsystem));
    }
}
