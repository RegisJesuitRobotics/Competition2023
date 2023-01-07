package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
    private SwerveUtils() {}

    /**
     * @param currentAngleRadians what the controller currently reads (radians)
     * @param targetAngleSetpointRadians the desired angle [-pi, pi)
     * @return the target angle in controller's scope
     */
    public static double calculateContinuousInputSetpoint(
            double currentAngleRadians, double targetAngleSetpointRadians) {
        double remainder = currentAngleRadians % (Math.PI * 2);
        double adjustedAngleSetpoint = targetAngleSetpointRadians + (currentAngleRadians - remainder);

        // We don't want to rotate over 180 degrees, so just rotate the other way (add a
        // full rotation)
        if (adjustedAngleSetpoint - currentAngleRadians > Math.PI) {
            adjustedAngleSetpoint -= Math.PI * 2;
        } else if (adjustedAngleSetpoint - currentAngleRadians < -Math.PI) {
            adjustedAngleSetpoint += Math.PI * 2;
        }

        return adjustedAngleSetpoint;
    }

    /**
     * Some controllers are not circular, so they can return something like (1, 1) which has a
     * magnitude of over 1 which could result in requesting too much from the system. This makes sure
     * that nothing goes over the maxMagnitude.
     *
     * @param translation the translation vector
     * @param maxMagnitude the maximum magnitude of the values
     * @return the normalized x and y value.
     */
    public static Translation2d applyCircleDeadZone(Translation2d translation, double maxMagnitude) {
        if (translation.getNorm() > maxMagnitude) {
            return translation.div(translation.getNorm()).times(maxMagnitude);
        }
        return translation;
    }

    public static boolean isChassisSpeedsZero(
            ChassisSpeeds chassisSpeeds, double allowedTranslation, double allowedOmega) {
        return Math.abs(chassisSpeeds.omegaRadiansPerSecond) < allowedOmega
                && Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) < allowedTranslation;
    }

    /**
     * @param swerveModuleState the state to copy
     * @return the copied state
     */
    public static SwerveModuleState copySwerveState(SwerveModuleState swerveModuleState) {
        return new SwerveModuleState(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }

    public static SwerveModuleState[] copySwerveStateArray(SwerveModuleState[] swerveModuleStates) {
        SwerveModuleState[] copied = new SwerveModuleState[swerveModuleStates.length];
        for (int i = 0; i < swerveModuleStates.length; i++) {
            copied[i] = copySwerveState(swerveModuleStates[i]);
        }
        return copied;
    }
}
