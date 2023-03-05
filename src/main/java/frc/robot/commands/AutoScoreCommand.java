package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderUtils;
import java.util.function.IntSupplier;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(IntSupplier scorePositionSupplier, SwerveDriveSubsystem driveSubsystem) {
        addCommands(new SimpleToPointCommand(
                () -> {
                    System.out.println(scorePositionSupplier.getAsInt());
                    return RaiderUtils.flipIfShould(
                            AutoScoreConstants.scoreFromLocations[scorePositionSupplier.getAsInt()]);
                },
                driveSubsystem));
    }
}
