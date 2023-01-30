package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class ConfigTimeout {
    private final double timeoutDuration;
    private final double startTime;

    public ConfigTimeout(double timeoutDuration) {
        this.timeoutDuration = timeoutDuration;

        startTime = Timer.getFPGATimestamp();
    }

    public boolean hasNotTimedOut() {
        return Timer.getFPGATimestamp() - startTime < timeoutDuration;
    }
}
