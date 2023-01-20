package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.telemetry.tunable.TunableFFGains;
import frc.robot.telemetry.tunable.TunablePIDGains;
import frc.robot.telemetry.tunable.TunableTrapezoidalProfileGains;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;

/** File containing all constants for the robot. */
public final class Constants {
    private Constants() {}

    public static final double DT = 0.02;

    public static class DriveTrainConstants {
        private DriveTrainConstants() {}

        public static final int NUM_MODULES = 4;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_GEAR_REDUCTION = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);

        public static final double STEER_GEAR_REDUCTION = 150.0 / 7.0;

        public static final boolean INVERT_GYRO = false;

        public static final double DRIVE_PEAK_CURRENT_LIMIT = 65.0;
        public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT = 35.0;
        public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;
        public static final double STEER_PEAK_CURRENT_LIMIT = 45.0;
        public static final double STEER_CONTINUOUS_CURRENT_LIMIT = 25.0;
        public static final double STEER_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;

        public static final double NOMINAL_VOLTAGE = 12.0;

        // For talons PID full output is 1023 except for all FF gains
        public static final TunablePIDGains DRIVE_VELOCITY_PID_GAINS =
                new TunablePIDGains("gains/drive", 0.1, 0.0, 0.0, MiscConstants.TUNING_MODE);

        public static final TunableFFGains DRIVE_VELOCITY_FF_GAINS =
                new TunableFFGains("gains/drive", 0.3346, 2.2549, 0.5731, MiscConstants.TUNING_MODE);

        public static final TunablePIDGains STEER_POSITION_PID_GAINS =
                new TunablePIDGains("gains/steer", 0.3, 0.0, 0.1, MiscConstants.TUNING_MODE);

        public static final double ACCEPTABLE_STEER_ERROR_RADIANS = Units.degreesToRadians(0.20);

        // Left right distance between center of wheels
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(24.75);

        // Front back distance between center of wheels
        public static final double WHEELBASE_METERS = Units.inchesToMeters(24.75);

        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
        };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        public static final double MOTOR_FREE_SPEED_RPM = 6380.0;
        public static final double MAX_VELOCITY_METERS_SECOND =
                (MOTOR_FREE_SPEED_RPM * WHEEL_DIAMETER_METERS * Math.PI) / (60.0 * DRIVE_GEAR_REDUCTION);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_SECOND = Math.PI * 4;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED =
                MAX_ANGULAR_VELOCITY_RADIANS_SECOND / 2.0;

        public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED = 10.0;
        public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED = 5.0 * Math.PI;
        public static final double TELEOP_MINIMUM_VELOCITY_METERS_SECOND = 0.10;
        public static final double TELEOP_MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND = 0.10;

        public static final String CAN_BUS = "rio";
        private static final SharedSwerveModuleConfiguration SHARED_SWERVE_MODULE_CONFIGURATION =
                new SharedSwerveModuleConfiguration(
                        CAN_BUS,
                        DRIVE_GEAR_REDUCTION,
                        STEER_GEAR_REDUCTION,
                        DRIVE_PEAK_CURRENT_LIMIT,
                        DRIVE_CONTINUOUS_CURRENT_LIMIT,
                        DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS,
                        STEER_PEAK_CURRENT_LIMIT,
                        STEER_CONTINUOUS_CURRENT_LIMIT,
                        STEER_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS,
                        NOMINAL_VOLTAGE,
                        WHEEL_DIAMETER_METERS,
                        MAX_VELOCITY_METERS_SECOND,
                        DRIVE_VELOCITY_PID_GAINS,
                        DRIVE_VELOCITY_FF_GAINS,
                        STEER_POSITION_PID_GAINS,
                        ACCEPTABLE_STEER_ERROR_RADIANS);

        public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                1, 5, 9, true, true, -1.22565065, false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                2, 6, 10, true, true, 1.30388367, false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                3, 7, 11, true, true, 1.37751475, false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                4, 8, 12, true, true, -2.73662173, false, SHARED_SWERVE_MODULE_CONFIGURATION);
    }

    public static class AutoConstants {
        public static final TunablePIDGains PATH_TRANSLATION_POSITION_GAINS =
                new TunablePIDGains("gains/pathXY", 2.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunablePIDGains PATH_ANGULAR_POSITION_PID_GAINS =
                new TunablePIDGains("gains/pathAngular", 1.5, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunablePIDGains SNAP_ANGULAR_POSITION_PID_GAINS =
                new TunablePIDGains("gains/snapAngular", 1.5, 0.0, 0.0, MiscConstants.TUNING_MODE);

        public static final TunablePIDGains AUTO_BALANCE_PID_GAINS =
                new TunablePIDGains("/gains/balance", 0, 0, 0, MiscConstants.TUNING_MODE);

        public static final TunableTrapezoidalProfileGains SNAP_ANGULAR_POSITION_TRAPEZOIDAL_GAINS =
                new TunableTrapezoidalProfileGains(
                        "gains/snapAngular",
                        DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND,
                        DriveTrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED,
                        MiscConstants.TUNING_MODE);

        public static final double MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED =
                DriveTrainConstants.MAX_VELOCITY_METERS_SECOND / 1.5;
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                DriveTrainConstants.MAX_VELOCITY_METERS_SECOND, MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static class TeleopConstants {
        private TeleopConstants() {}

        public static final boolean OPEN_LOOP_DRIVETRAIN = false;
    }

    public static class FieldConstants {
        private FieldConstants() {}

        // TODO Validate
        public static final Translation2d[] CHARGE_STATION_CORNERS = new Translation2d[4];
        // TODO
        public static final Translation2d CENTER_OF_CHARGE_STATION = new Translation2d();
    }

    public static class MiscConstants {
        private MiscConstants() {}

        public static final int[] USED_CONTROLLER_PORTS = {0};
        public static final boolean TUNING_MODE = true;
    }
}
