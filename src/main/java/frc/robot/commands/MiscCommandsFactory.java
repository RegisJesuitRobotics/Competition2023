package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants.Grids;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.utils.LiftExtensionKinematics;

public class MiscCommandsFactory {
    private MiscCommandsFactory() {}

    public static Command clawClearOfMidCubeCommand(
            LiftSubsystem liftSubsystem, ExtensionSubsystem extensionSubsystem) {
        return new WaitUntilCommand(() -> {
            Translation2d clawPosition = LiftExtensionKinematics.liftExtensionPositionToClawPosition(
                    liftSubsystem.getArmAngle(), extensionSubsystem.getPosition());
            return clawPosition.getY() >= Grids.midCubeZ + Units.inchesToMeters(6.0);
        });
    }
}
