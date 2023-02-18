package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Homeable;

public class HomeHomeableCommand extends CommandBase {
    private final Homeable homeable;
    private final double homeVoltage;
    private final double homeCurrent;

    private final Debouncer debouncer = new Debouncer(0.1);

    public HomeHomeableCommand(double homeVoltage, double homeCurrent, Homeable homeable) {
        this.homeVoltage = homeVoltage;
        this.homeCurrent = homeCurrent;
        this.homeable = homeable;

        addRequirements(homeable);
    }

    @Override
    public void initialize() {
        // Reset debounce
        debouncer.calculate(false);
    }

    @Override
    public void execute() {
        homeable.setVoltage(homeVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            homeable.setInHome();
        }
        homeable.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(Math.abs(homeable.getCurrent()) >= homeCurrent);
    }
}
