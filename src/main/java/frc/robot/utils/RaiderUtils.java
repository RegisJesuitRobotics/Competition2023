package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

public class RaiderUtils {
    private RaiderUtils() {}

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
}
