package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DualHomeable extends Subsystem {
    void setInHome();

    double getLeftCurrent();

    double getRightCurrent();

    void setVoltage(double leftVoltage, double rightVoltage);
}
