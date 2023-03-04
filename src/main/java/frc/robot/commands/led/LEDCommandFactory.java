package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.utils.RaiderCommands;
import frc.robot.utils.led.AlternatePattern;
import frc.robot.utils.led.RandomColorsPattern;
import frc.robot.utils.led.SlidePattern;
import frc.robot.utils.led.SolidPattern;

public class LEDCommandFactory {
    private LEDCommandFactory() {}

    public static CommandBase solidColorCommand(Color color, LEDSubsystem ledSubsystem) {
        return RaiderCommands.startNoEnd(() -> ledSubsystem.setAllPattern(new SolidPattern(color)), ledSubsystem)
                .ignoringDisable(true);
    }

    public static CommandBase alternateColorCommand(
            double period, Color color1, Color color2, LEDSubsystem ledSubsystem) {
        return RaiderCommands.startNoEnd(
                        () -> ledSubsystem.setAllPattern(new AlternatePattern(period, color1, color2)), ledSubsystem)
                .ignoringDisable(true);
    }

    public static CommandBase slideAlternateColorCommand(
            double period, Color color1, Color color2, LEDSubsystem ledSubsystem) {
        return RaiderCommands.startNoEnd(
                        () -> ledSubsystem.setAllPattern(new AlternatePattern(
                                period,
                                new SlidePattern(period / 2.0, color1, color2),
                                new SlidePattern(period / 2.0, color2, color1))),
                        ledSubsystem)
                .ignoringDisable(true);
    }

    /**
     * @param partyLevel the hype of the party from 1-10
     * @param ledSubsystem the LED subsystem
     * @return the command
     */
    public static CommandBase partyModeCommand(double partyLevel, LEDSubsystem ledSubsystem) {
        double period = 1.0 / partyLevel * 2.0;
        return RaiderCommands.startNoEnd(
                        () -> ledSubsystem.setAllPattern(new AlternatePattern(
                                period, new RandomColorsPattern(period), new SolidPattern(Color.kBlack))),
                        ledSubsystem)
                .ignoringDisable(true);
    }
}
