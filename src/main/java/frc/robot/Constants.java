package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.telemetry.tunable.TunableFFGains;
import frc.robot.telemetry.tunable.TunablePIDGains;
import frc.robot.telemetry.tunable.TunableTrapezoidalProfileGains;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;

/** File containing all constants for the robot. */
public final class Constants {
    private Constants() {}

    public static class VisionConstants {
        private VisionConstants() {}

        // TODO: update once cameras are mounted
        public static final Transform3d FRONT_CAMERA_LOCATION = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(2.26271), Units.inchesToMeters(11.55917), Units.inchesToMeters(35.851)),
                new Rotation3d(0, Units.degreesToRadians(5.0), 0));

        public static final String FRONT_CAMERA_NAME = "FrontCamera";
    }

    public static final double DT = 0.02;

    public static class DriveTrainConstants {
        private DriveTrainConstants() {}

        public static final int NUM_MODULES = 4;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_GEAR_REDUCTION = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);

        public static final double STEER_GEAR_REDUCTION = 150.0 / 7.0;

        public static final boolean INVERT_GYRO = true;

        public static final double DRIVE_PEAK_CURRENT_LIMIT = 65.0;
        public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT = 35.0;
        public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;
        public static final double STEER_PEAK_CURRENT_LIMIT = 45.0;
        public static final double STEER_CONTINUOUS_CURRENT_LIMIT = 25.0;
        public static final double STEER_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;

        public static final double NOMINAL_VOLTAGE = 12.0;

        // For talons PID full output is 1023 except for all FF gains
        public static final TunablePIDGains DRIVE_VELOCITY_PID_GAINS =
                new TunablePIDGains("/gains/drive", 0.042758, 0.0, 0.0, MiscConstants.TUNING_MODE);

        public static final TunableFFGains DRIVE_VELOCITY_FF_GAINS =
                new TunableFFGains("/gains/drive", 0.1152, 2.2639, 0.22216, MiscConstants.TUNING_MODE);

        public static final TunablePIDGains STEER_POSITION_PID_GAINS =
                new TunablePIDGains("/gains/steer", 0.3, 0.0, 0.1, MiscConstants.TUNING_MODE);

        public static final double ACCEPTABLE_STEER_ERROR_RADIANS = Units.degreesToRadians(0.20);

        // Left right distance between center of wheels
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(28.616);

        // Front back distance between center of wheels
        public static final double WHEELBASE_METERS = Units.inchesToMeters(20.75);

        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
            new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
            new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
            new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)
        };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        public static final double MOTOR_FREE_SPEED_RPM = 6380.0;
        public static final double MAX_VELOCITY_METERS_SECOND =
                (MOTOR_FREE_SPEED_RPM * WHEEL_DIAMETER_METERS * Math.PI) / (60.0 * DRIVE_GEAR_REDUCTION);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_SECOND = Math.PI * 4;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED =
                MAX_ANGULAR_VELOCITY_RADIANS_SECOND / 2.0;

        public static final String CAN_BUS = "canivore";
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
                1, 5, 9, true, true, Units.degreesToRadians(-72.685547), false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                2, 6, 10, true, true, Units.degreesToRadians(75.673828), false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                3, 7, 11, true, true, Units.degreesToRadians(78.044531), false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                4, 8, 12, true, true, Units.degreesToRadians(-156.621094), false, SHARED_SWERVE_MODULE_CONFIGURATION);
    }

    public static class AutoConstants {
        public static final TunablePIDGains PATH_TRANSLATION_POSITION_GAINS =
                new TunablePIDGains("/gains/pathXY", 2.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunablePIDGains PATH_ANGULAR_POSITION_PID_GAINS =
                new TunablePIDGains("/gains/pathAngular", 1.5, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunablePIDGains SNAP_ANGULAR_POSITION_PID_GAINS =
                new TunablePIDGains("/gains/snapAngular", 1.5, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains SNAP_ANGULAR_POSITION_TRAPEZOIDAL_GAINS =
                new TunableTrapezoidalProfileGains(
                        "/gains/snapAngular",
                        DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND,
                        DriveTrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED,
                        MiscConstants.TUNING_MODE);
        public static final double MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED =
                DriveTrainConstants.MAX_VELOCITY_METERS_SECOND / 1.5;
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                DriveTrainConstants.MAX_VELOCITY_METERS_SECOND, MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static class ClawConstants {
        public static final int[] SOLENOID_PORTS = {6, 3};
    }

    // TODO: change names when theres a cad for intake and change ports
    public static class IntakeConstants {
        public static final int[] UP_DOWN_SOLENOID_PORTS = {5, 2};
        public static final int[] LEFT_RIGHT_SOLENOID_PORTS = {4, 1};
    }

    public static class LEDConstants {
        public static final int PWM_PORT = 0;
        public static final int BUFFER_SIZE = 50;
        // HTML "Gold"
        public static final int[] YELLOW_RGB = {255, 215, 0};
        // HTML "RebeccaPurple"
        public static final int[] PURPLE_RGB = {102, 51, 153};
    }

    public static class TeleopConstants {
        private TeleopConstants() {}

        public static final boolean OPEN_LOOP_DRIVETRAIN = false;
        public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED = 10.0;
        public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED = 5.0 * Math.PI;
        public static final double MINIMUM_VELOCITY_METERS_SECOND = 0.10;
        public static final double MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND = 0.10;
    }

    public static class MiscConstants {

        private MiscConstants() {}

        public static final int[] USED_CONTROLLER_PORTS = {0};
        public static final boolean TUNING_MODE = true;

        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;
        public static final int POWER_MODULE_ID = 1;
    }
}
