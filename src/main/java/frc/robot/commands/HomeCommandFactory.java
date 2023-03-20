package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class HomeCommandFactory {
    public static Command homeExtensionCommand(ExtensionSubsystem extensionSubsystem) {
        return new HomeHomeableCommand(
                ExtensionConstants.HOME_VOLTAGE, ExtensionConstants.HOME_CURRENT, extensionSubsystem);
    }

    public static Command homeLiftCommand(LiftSubsystem liftSubsystem) {
        return new HomeHomeableCommand(LiftConstants.HOME_VOLTAGE, LiftConstants.HOME_CURRENT, liftSubsystem);
    }
}
