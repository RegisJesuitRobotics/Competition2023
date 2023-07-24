package frc.robot.utils;

import com.revrobotics.REVLibError;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;

public record SwerveModuleConfiguration(
        int driveMotorPort,
        int steerMotorPort,
        int steerEncoderPort,
        boolean driveMotorInverted,
        boolean steerMotorInverted,
        double offsetRadians,
        boolean steerEncoderInverted,
        SharedSwerveModuleConfiguration sharedConfiguration) {
    /** This is all the options that are not module specific */
    public record SharedSwerveModuleConfiguration(
            String canBus,
            double driveGearRatio,
            double steerGearRatio,
            double drivePeakCurrentLimit,
            double driveContinuousCurrentLimit,
            double drivePeakCurrentDurationSeconds,
            double steerPeakCurrentLimit,
            double steerContinuousCurrentLimit,
            double steerPeakCurrentDurationSeconds,
            double nominalVoltage,
            double wheelDiameterMeters,
            double openLoopMaxSpeed,
            double steerClosedLoopRamp,
            double driveClosedLoopRamp,
            double driveOpenLoopRamp,
            double maxSteerVoltage,
            TunablePIDGains driveVelocityPIDGains,
            TunableFFGains driveVelocityFFGains,
            TunablePIDGains steerPositionPIDGains,
            double allowableSteerErrorRadians) {}
}
