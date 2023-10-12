package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.extension.SetExtensionPositionCommand;
import frc.robot.commands.lift.SetLiftPositionCommand;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class PositionClawCommand extends ParallelCommandGroup {
    public PositionClawCommand(
            Pair<Rotation2d, Double> positions, LiftSubsystem liftSubsystem, ExtensionSubsystem extensionSubsystem) {
        this(positions.getFirst(), positions.getSecond(), liftSubsystem, extensionSubsystem);
    }

    public PositionClawCommand(
            Rotation2d liftAngle,
            double extensionPosition,
            LiftSubsystem liftSubsystem,
            ExtensionSubsystem extensionSubsystem) {
        addCommands(Commands.parallel(
                new SetLiftPositionCommand(liftAngle, true, liftSubsystem),
                Commands.sequence(
                        Commands.deadline(
                                new WaitUntilCommand(() -> liftSubsystem.getEstimatedTimeForPosition(liftAngle)
                                                - extensionSubsystem.getEstimatedTimeForPosition(extensionPosition)
                                        < 0.2),
                                new SetExtensionPositionCommand(Units.inchesToMeters(0.5), extensionSubsystem)),
                        new SetExtensionPositionCommand(extensionPosition, extensionSubsystem))));
    }
}
