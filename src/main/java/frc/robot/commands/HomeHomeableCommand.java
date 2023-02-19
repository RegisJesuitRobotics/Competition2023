package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Homeable;

public class HomeHomeableCommand extends CommandBase {
    private final Homeable homeable;
    private final double homeVoltage;
    private final double homeCurrent;

    private final MedianFilter currentFilter = new MedianFilter(4);

    public HomeHomeableCommand(double homeVoltage, double homeCurrent, Homeable homeable) {
        this.homeVoltage = homeVoltage;
        this.homeCurrent = homeCurrent;
        this.homeable = homeable;

        addRequirements(homeable);
    }

    @Override
    public void initialize() {
        currentFilter.reset();
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
        return currentFilter.calculate(homeable.getCurrent()) > homeCurrent;
    }
}
