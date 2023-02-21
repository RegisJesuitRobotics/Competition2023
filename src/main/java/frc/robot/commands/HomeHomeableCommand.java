package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DualHomeable;

public class HomeHomeableCommand extends CommandBase {
    private final DualHomeable dualHomeable;
    private final double homeVoltage;
    private final double homeCurrent;

    private final MedianFilter leftFilter = new MedianFilter(4);
    private final MedianFilter rightFilter = new MedianFilter(4);

    private boolean done = false;

    public HomeHomeableCommand(double homeVoltage, double homeCurrent, DualHomeable dualHomeable) {
        this.homeVoltage = homeVoltage;
        this.homeCurrent = homeCurrent;
        this.dualHomeable = dualHomeable;

        addRequirements(dualHomeable);
    }

    @Override
    public void initialize() {
        done = false;
        leftFilter.reset();
        rightFilter.reset();
    }

    @Override
    public void execute() {
        double leftVoltage, rightVoltage;
        if (leftFilter.calculate(dualHomeable.getLeftCurrent()) > homeCurrent) {
            leftVoltage = 0.0;
            done = true;
        } else {
            leftVoltage = homeVoltage;
            done = false;
        }
        if (rightFilter.calculate(dualHomeable.getRightCurrent()) > homeCurrent) {
            rightVoltage = 0.0;
        } else {
            rightVoltage = homeVoltage;
            done = false;
        }

        dualHomeable.setVoltage(leftVoltage, rightVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            dualHomeable.setInHome();
        }
        dualHomeable.setVoltage(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
