package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
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
        addCommands(new ProxyCommand(() -> {
            double liftTime = liftSubsystem.getEstimatedTimeForPosition(liftAngle);
            double extensionTime = extensionSubsystem.getEstimatedTimeForPosition(extensionPosition);
            if (extensionTime >= liftTime) {
                return Commands.parallel(
                        new SetLiftPositionCommand(liftAngle, liftSubsystem),
                        new SetExtensionPositionCommand(extensionPosition, extensionSubsystem));
            } else {
                // "Tuck" extension until it will get to our desired position when lift does
                return Commands.parallel(
                        new SetLiftPositionCommand(liftAngle, liftSubsystem),
                        Commands.sequence(
                                Commands.deadline(
                                        new WaitUntilCommand(() -> liftSubsystem.getEstimatedTimeForPosition(liftAngle)
                                                        - extensionSubsystem.getEstimatedTimeForPosition(
                                                                extensionPosition)
                                                > 0.2),
                                        new SetExtensionPositionCommand(0.0, extensionSubsystem)),
                                new SetExtensionPositionCommand(extensionPosition, extensionSubsystem)));
            }
        }));
    }
}
