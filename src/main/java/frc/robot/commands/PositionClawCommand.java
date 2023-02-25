package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.extension.SetExtensionPositionCommand;
import frc.robot.commands.lift.SetLiftPositionCommand;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class PositionClawCommand extends ParallelCommandGroup {
    public PositionClawCommand(
            Pair<Rotation2d, Double> positions, boolean liftFirst, LiftSubsystem liftSubsystem, ExtensionSubsystem extensionSubsystem) {
        this(positions.getFirst(), positions.getSecond(), liftFirst, liftSubsystem, extensionSubsystem);
    }

    public PositionClawCommand(
            Rotation2d liftAngle,
            double extensionPosition,
            boolean liftFirst,
            LiftSubsystem liftSubsystem,
            ExtensionSubsystem extensionSubsystem) {
        if (liftFirst) {
        addCommands(
                new SetLiftPositionCommand(liftAngle, liftSubsystem),
                Commands.sequence(
                        new ProxyCommand(() -> {
                            double waitTime = Math.max(
                                    0.2,
                                    liftSubsystem.getEstimatedTimeForPosition(liftAngle)
                                            - extensionSubsystem.getEstimatedTimeForPosition(extensionPosition));
                            return new WaitCommand(waitTime);
                        }),
                        new SetExtensionPositionCommand(extensionPosition, extensionSubsystem)));
        } else {
            addCommands(
                new SetExtensionPositionCommand(extensionPosition, extensionSubsystem),
                Commands.sequence(
                    new ProxyCommand(() -> {
                        double waitTime = Math.max(
                                0.2,
                                extensionSubsystem.getEstimatedTimeForPosition(extensionPosition)) / 2.0;
                        return new WaitCommand(waitTime);
                    }),
                    new SetLiftPositionCommand(liftAngle, liftSubsystem)
                )
            );
        }
    }
}
