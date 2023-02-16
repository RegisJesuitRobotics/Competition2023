package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.FieldConstants.Community;
import frc.robot.FieldConstants.Grids;
import frc.robot.telemetry.tunable.gains.TunableArmFFGains;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.tunable.gains.TunableTrapezoidalProfileGains;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;
import frc.robot.utils.geometry.Rectangle;

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
        private AutoConstants() {}

        public static final double MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED =
                DriveTrainConstants.MAX_VELOCITY_METERS_SECOND / 2.0;
        public static final double MAX_AUTO_VELOCITY_METERS_SECOND =
                DriveTrainConstants.MAX_VELOCITY_METERS_SECOND / 1.25;
        public static final TrajectoryConfig TRAJECTORY_CONSTRAINTS =
                new TrajectoryConfig(MAX_AUTO_VELOCITY_METERS_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TunablePIDGains TRANSLATION_POSITION_GAINS =
                new TunablePIDGains("/gains/driveXY", 2.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains TRANSLATION_POSITION_TRAPEZOIDAL_GAINS =
                new TunableTrapezoidalProfileGains(
                        "/gains/driveXY",
                        MAX_AUTO_VELOCITY_METERS_SECOND,
                        MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED,
                        MiscConstants.TUNING_MODE);
        public static final TunablePIDGains ANGULAR_POSITION_PID_GAINS =
                new TunablePIDGains("/gains/driveAngular", 1.5, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains ANGULAR_POSITION_TRAPEZOIDAL_GAINS =
                new TunableTrapezoidalProfileGains(
                        "/gains/driveAngular",
                        DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND,
                        DriveTrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED,
                        MiscConstants.TUNING_MODE);
    }

    public static class LiftConstants {
        private LiftConstants() {}

        public static final int LEFT_MOTOR_PORT = 1;
        public static final int RIGHT_MOTOR_PORT = 2;

        public static final boolean INVERT_LEADER = false;
        public static final boolean INVERT_FOLLOWER_FROM_LEADER = false;

        public static final double HORIZONTAL_BAR_LENGTH = Units.inchesToMeters(32.0);
        public static final double VERTICAL_BAR_HEIGHT_FROM_FLOOR = Units.inchesToMeters(40.724);
        public static final double HORIZONTAL_BAR_TO_CLAW = Units.inchesToMeters(5.274);

        public static final Translation2d TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL =
                new Translation2d(Units.inchesToMeters(-1.0), Units.inchesToMeters(-5.0));

        public static final double GEAR_REDUCTION = 5.0 * 5.0 * 5.0 * 5.0;

        // TODO: Update this
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(165.0);

        public static final TunablePIDGains PID_GAINS =
                new TunablePIDGains("/gains/lifter", 4.7362, 0.0, 3.8901, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
                new TunableTrapezoidalProfileGains(
                        "/gains/lifter", Math.PI / 2, Math.PI / 2, MiscConstants.TUNING_MODE);
        public static final TunableArmFFGains FF_GAINS =
                new TunableArmFFGains("/gains/lifter", 0.12526, 0.15687, 10.508, 0.19378, MiscConstants.TUNING_MODE);

        public static final int STALL_CURRENT_LIMIT = 10;
        public static final int FREE_CURRENT_LIMIT = 50;
    }

    public static class ExtensionConstants {
        private ExtensionConstants() {}

        public static final int LEFT_MOTOR_PORT = 3;
        public static final int RIGHT_MOTOR_PORT = 4;

        public static final boolean INVERT_LEADER = false;
        public static final boolean INVERT_FOLLOWER_FROM_LEADER = true;

        public static final int STALL_CURRENT_LIMIT = 10;
        public static final int FREE_CURRENT_LIMIT = 20;

        public static final double GEAR_REDUCTION = 3.0 * 4.0 * 5.0;
        public static final double ROLLER_DIAMETER_METERS = Units.inchesToMeters(0.5);

        // Offset from claw to center of robot when at "0"
        public static final double X_OFFSET_METERS = Units.inchesToMeters(-1);

        public static final TunablePIDGains PID_GAINS =
                new TunablePIDGains("/gains/extension", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
                new TunableTrapezoidalProfileGains("/gains/extension", 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableFFGains FF_GAINS =
                new TunableFFGains("/gains/extension", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    }

    public static class TeleopConstants {
        private TeleopConstants() {}

        public static final boolean OPEN_LOOP_DRIVETRAIN = false;
        public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED = 10.0;
        public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED = 5.0 * Math.PI;
        public static final double MINIMUM_VELOCITY_METERS_SECOND = 0.10;
        public static final double MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND = 0.10;

        public static final double DRIVER_TAKE_CONTROL_THRESHOLD = 0.2;
    }

    public static class AutoScoreConstants {
        private AutoScoreConstants() {}

        public enum ScoreLevel {
            LOW,
            MID,
            HIGH
        }

        public enum ScorePiece {
            CUBE,
            CONE
        }

        private static final double BUMPER_OFFSET_FROM_LOW_EDGE = Units.inchesToMeters(5.0);
        private static final double CONE_X_OFFSET = Units.inchesToMeters(0.0);
        private static final double CUBE_X_OFFSET = Units.inchesToMeters(0.0);
        private static final double CONE_Z_OFFSET = Units.inchesToMeters(4.0);
        private static final double CUBE_Z_OFFSET = Units.inchesToMeters(3.0);
        private static final double GROUND_OFFSET = Units.inchesToMeters(6.0);

        public static final double ROBOT_SCORING_X =
                Grids.outerX + BUMPER_OFFSET_FROM_LOW_EDGE + (MiscConstants.FULL_ROBOT_LENGTH_METERS / 2.0);

        public static final Translation2d CONE_HIGH =
                new Translation2d(ROBOT_SCORING_X - Grids.highX + CONE_X_OFFSET, Grids.highConeZ + CONE_Z_OFFSET);
        public static final Translation2d CONE_MID =
                new Translation2d(ROBOT_SCORING_X - Grids.midX + CONE_X_OFFSET, Grids.midConeZ + CONE_Z_OFFSET);
        public static final Translation2d CONE_LOW =
                new Translation2d(ROBOT_SCORING_X - Grids.lowX + CONE_X_OFFSET, GROUND_OFFSET);
        public static final Translation2d CUBE_HIGH =
                new Translation2d(ROBOT_SCORING_X - Grids.highX + CUBE_X_OFFSET, Grids.highCubeZ + CUBE_Z_OFFSET);
        public static final Translation2d CUBE_MID =
                new Translation2d(ROBOT_SCORING_X - Grids.midX + CUBE_X_OFFSET, Grids.midCubeZ + CUBE_Z_OFFSET);
        public static final Translation2d CUBE_LOW =
                new Translation2d(ROBOT_SCORING_X - Grids.lowX + CUBE_X_OFFSET, GROUND_OFFSET);

        public static final Pose2d[] scoreFromLocations = new Pose2d[Grids.highTranslations.length];

        static {
            for (int i = 0; i < scoreFromLocations.length; i++) {
                scoreFromLocations[i] = new Pose2d(
                        new Translation2d(ROBOT_SCORING_X, Grids.lowTranslations[i].getY()),
                        Rotation2d.fromDegrees(180.0));
            }
        }

        public static final Rectangle ALLOWED_SCORING_AREA = new Rectangle(
                new Translation2d(Community.innerX, Community.rightY),
                new Translation2d(
                        Community.chargingStationInnerX - (MiscConstants.LONGEST_SIDE_METERS / 2.0), Community.leftY));
    }

    public static class MiscConstants {

        private MiscConstants() {}

        public static final int[] USED_CONTROLLER_PORTS = {0};
        public static final boolean TUNING_MODE = true;

        public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;
        public static final int POWER_MODULE_ID = 1;
        public static final double CONFIGURATION_TIMEOUT_SECONDS = 5.0;

        public static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(0.75 + 2.3);
        // With bumpers
        public static final double FULL_ROBOT_LENGTH_METERS = Units.inchesToMeters(26.0) + (BUMPER_WIDTH_METERS * 2);
        public static final double FULL_ROBOT_WIDTH_METERS = Units.inchesToMeters(33.866) + (BUMPER_WIDTH_METERS * 2);

        public static final double LONGEST_SIDE_METERS = Math.max(FULL_ROBOT_LENGTH_METERS, FULL_ROBOT_WIDTH_METERS);
    }
}
