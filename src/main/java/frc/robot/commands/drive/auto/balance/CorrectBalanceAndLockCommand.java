package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class CorrectBalanceAndLockCommand extends SequentialCommandGroup {
    public CorrectBalanceAndLockCommand(SwerveDriveSubsystem driveSubsystem) {
        addCommands(
                new CorrectBalancePart1Command(driveSubsystem),
                new SimpleToPointCommand(
                        () -> {
                            double translation = CorrectBalancePart1Command.getAngle(
                                                    driveSubsystem.getPose().getRotation(),
                                                    driveSubsystem.getPitchRadians(),
                                                    driveSubsystem.getRollRadians())
                                            > 0
                                    ? -0.85
                                    : 0.85;
                            return new Pose2d(
                                    driveSubsystem.getPose().getTranslation().plus(new Translation2d(translation, 0.0)),
                                    driveSubsystem.getPose().getRotation());
                        },
                        driveSubsystem),
                new LockModulesCommand(driveSubsystem));
    }
}
