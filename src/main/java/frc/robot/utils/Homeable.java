package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Homeable extends Subsystem {
    void setInHome();

    double getCurrent();

    void setVoltage(double voltage);
}
