package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class RaiderMathUtils {
    private RaiderMathUtils() {}

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

    /**
     * @param value the value
     * @param pow   the power to put the value to
     * @return      the value returned by with sign copied
     */
    public static double signCopyPow(double value, double pow) {
        return Math.signum(value) * Math.pow(Math.abs(value), pow);
    }

    public static double deadZoneAndSquareJoystick(double value) {
        return signCopyPow(MathUtil.applyDeadband(value, 0.03), 2);
    }

    public static ChassisSpeeds correctForSwerveSkew(ChassisSpeeds speeds) {
        // From Team 254
        Pose2d robotVelocityPose = new Pose2d(
                speeds.vxMetersPerSecond * Constants.DT,
                speeds.vyMetersPerSecond * Constants.DT,
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.DT));
        Twist2d velocityTwist = new Pose2d().log(robotVelocityPose);
        return new ChassisSpeeds(
                velocityTwist.dx / Constants.DT, velocityTwist.dy / Constants.DT, velocityTwist.dtheta / Constants.DT);
    }
}
