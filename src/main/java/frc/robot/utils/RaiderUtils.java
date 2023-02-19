package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
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

    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }

    public static Pose2d flipIfShould(Pose2d pose) {
        if (shouldFlip()) {
            return allianceFlip(pose);
        }
        return pose;
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return new Pose2d(
                new Translation2d(
                        FieldConstants.fieldLength - pose.getTranslation().getX(),
                        pose.getTranslation().getY()),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    }
}
