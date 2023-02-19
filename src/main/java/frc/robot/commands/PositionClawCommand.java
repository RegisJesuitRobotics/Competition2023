package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.extension.SetExtensionPositionCommand;
import frc.robot.commands.lift.SetLiftPositionCommand;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.utils.LiftExtensionKinematics;

public class PositionClawCommand extends ParallelCommandGroup {
    public PositionClawCommand(
            Translation2d clawTranslation, LiftSubsystem liftSubsystem, ExtensionSubsystem extensionSubsystem) {
        Pair<Rotation2d, Double> liftExtensionGoal =
                LiftExtensionKinematics.clawPositionToLiftExtensionPosition(clawTranslation);
        addCommands(
                new SetLiftPositionCommand(liftExtensionGoal.getFirst(), liftSubsystem),
                new SetExtensionPositionCommand(liftExtensionGoal.getSecond(), extensionSubsystem));
    }
}
