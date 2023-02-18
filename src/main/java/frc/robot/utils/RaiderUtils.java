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

    public static boolean anyTrue(boolean[] array) {
        for (boolean bool : array) {
            if (bool) {
                return true;
            }
        }
        return false;
    }
}

    /**
     * @param code the error code
     * @return true if there was a fault
     */
    public static boolean checkCTREError(ErrorCode code) {
        return code != ErrorCode.OK;
    }

    public static boolean checkRevError(REVLibError code) {
        return code != REVLibError.kOk;
    }