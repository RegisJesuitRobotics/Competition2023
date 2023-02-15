package frc.robot.utils;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RaiderUtils {
    public static int getSolenoidValueToInt(Value value) {
        return switch (value) {
            case kOff -> 0;
            case kForward -> 1;
            case kReverse -> 2;
        };
    }
}
