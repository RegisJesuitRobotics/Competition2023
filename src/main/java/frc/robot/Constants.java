package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.FieldConstants.Community;
import frc.robot.FieldConstants.Grids;
import frc.robot.FieldConstants.LoadingZone;
import frc.robot.telemetry.tunable.gains.TunableArmFFGains;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.tunable.gains.TunableTrapezoidalProfileGains;
import frc.robot.utils.LiftExtensionKinematics;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;
import frc.robot.utils.geometry.Rectangle;
import java.util.Collections;
import java.util.List;

/** File containing all constants for the robot. */
public final class Constants {
    private Constants() {}

    public static class VisionConstants {
        private VisionConstants() {}

        public static final Transform3d FRONT_CAMERA_LOCATION = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(1.5 + 1.268095),
                        Units.inchesToMeters((20.75 / 2.0) + 1.738934 - (1.276220 / 2.0)),
                        Units.inchesToMeters(33.0 + 4.0 - 0.969)),
                new Rotation3d(
                        Units.degreesToRadians(0.0), Units.degreesToRadians(13.5 - 2.5), Units.degreesToRadians(0.0)));
        public static final Transform3d BACK_CAMERA_LOCATION = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(1.5 - 1.268095),
                        Units.inchesToMeters((20.75 / 2.0) + 1.738934 - (1.276220 / 2.0)),
                        Units.inchesToMeters(33.0 + 4.0 - 0.969)),
                new Rotation3d(
                        Units.degreesToRadians(0.0),
                        Units.degreesToRadians(13.5 + 2.5),
                        Units.degreesToRadians(180.0)));

        public static final String FRONT_CAMERA_NAME = "FrontCamera";
        public static final String BACK_CAMERA_NAME = "BackCamera";

        public static final double POSE_AMBIGUITY_CUTOFF = 0.05;
        public static final double DISTANCE_CUTOFF = 4.0;
    }

    public static final double DT = 0.02;

    public static class DriveTrainConstants {
        private DriveTrainConstants() {}

        public static final int NUM_MODULES = 4;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.875);
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
                new TunablePIDGains("/gains/drive", 0.3, 0.0, 0.0, MiscConstants.TUNING_MODE);

        public static final TunableFFGains DRIVE_VELOCITY_FF_GAINS =
                new TunableFFGains("/gains/drive", 0.2776, 2.32302894, 0.31227, MiscConstants.TUNING_MODE);

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
        public static final double MAX_VELOCITY_METERS_SECOND = 4.2;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_SECOND = Math.PI * 3;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED =
                MAX_ANGULAR_VELOCITY_RADIANS_SECOND / 2.0;

        public static final double STEER_CLOSED_LOOP_RAMP = 0.03;
        public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;
        public static final double DRIVE_OPEN_LOOP_RAMP = 0.03;
        public static final double MAX_STEER_VOLTAGE = 12.0;

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
                        STEER_CLOSED_LOOP_RAMP,
                        DRIVE_CLOSED_LOOP_RAMP,
                        DRIVE_OPEN_LOOP_RAMP,
                        MAX_STEER_VOLTAGE,
                        DRIVE_VELOCITY_PID_GAINS,
                        DRIVE_VELOCITY_FF_GAINS,
                        STEER_POSITION_PID_GAINS,
                        ACCEPTABLE_STEER_ERROR_RADIANS);

        public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                1, 5, 9, true, true, Units.degreesToRadians(-69.685547), false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                2, 6, 10, true, true, Units.degreesToRadians(75.673828), false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                3, 7, 11, true, true, Units.degreesToRadians(78.044531), false, SHARED_SWERVE_MODULE_CONFIGURATION);

        public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                4, 8, 12, true, true, Units.degreesToRadians(-156.621094), false, SHARED_SWERVE_MODULE_CONFIGURATION);
    }

    public static class AutoConstants {
        private AutoConstants() {}

        public static final double MAX_AUTO_VELOCITY_METERS_SECOND = 2.5;
        public static final double MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;

        public static final PathConstraints TRAJECTORY_CONSTRAINTS =
                new PathConstraints(MAX_AUTO_VELOCITY_METERS_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final PathConstraints SLOW_TRAJECTORY_CONSTRAINTS = new PathConstraints(2, 1.5);
        public static final PathConstraints VERY_SLOW_TRAJECTORY_CONSTRAINTS = new PathConstraints(1.5, 1);

        public static final double MAX_AUTO_ANGULAR_VELOCITY_RADIANS_SECOND = Math.PI * 2;
        public static final double MAX_AUTO_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED = Math.PI;

        public static final TunablePIDGains TRANSLATION_POSITION_GAINS =
                new TunablePIDGains("/gains/driveXY", 2.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains TRANSLATION_POSITION_TRAPEZOIDAL_GAINS =
                new TunableTrapezoidalProfileGains(
                        "/gains/driveXY",
                        MAX_AUTO_VELOCITY_METERS_SECOND,
                        MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED,
                        MiscConstants.TUNING_MODE);
        public static final TunablePIDGains ANGULAR_POSITION_PID_GAINS =
                new TunablePIDGains("/gains/driveAngular", 1.1, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains ANGULAR_POSITION_TRAPEZOIDAL_GAINS =
                new TunableTrapezoidalProfileGains(
                        "/gains/driveAngular",
                        MAX_AUTO_ANGULAR_VELOCITY_RADIANS_SECOND,
                        MAX_AUTO_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED,
                        MiscConstants.TUNING_MODE);

        public static final double AUTO_BALANCE_TOLERANCE_RADIANS = Units.degreesToRadians(1.0);
        public static final double AUTO_BALANCE_SPEED_METERS_SECOND = 0.5;
    }

    public static class ClawConstants {
        public static final int[] SOLENOID_PORTS = {6, 3};
    }

    public static class FlipperConstants {
        public static final int[] UP_DOWN_SOLENOID_PORTS = {5, 2};
        public static final int[] LEFT_RIGHT_SOLENOID_PORTS = {4, 1};

        public static final double UP_DOWN_DOWN_TIME = 0.5;
        public static final double UP_DOWN_UP_TIME = 0.5;
    }

    public static class LiftConstants {
        private LiftConstants() {}

        public static final int LEFT_MOTOR_PORT = 1;
        public static final int RIGHT_MOTOR_PORT = 2;

        public static final boolean INVERT_LEFT = false;
        public static final boolean INVERT_RIGHT = false;

        public static final double HORIZONTAL_BAR_LENGTH = Units.inchesToMeters(32.0);
        public static final double VERTICAL_BAR_HEIGHT_FROM_FLOOR = Units.inchesToMeters(43.5);
        public static final double HORIZONTAL_BAR_TO_CLAW = Units.inchesToMeters(7.0);

        public static final Translation2d TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL =
                new Translation2d(Units.inchesToMeters(-1.0), Units.inchesToMeters(-5.0));

        public static final double GEAR_REDUCTION = 5.0 * 5.0 * 5.0 * 4.0;

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-87.0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(40.0);

        public static final int ABSOLUTE_ENCODER_PORT = 0;
        public static final double ABSOLUTE_ENCODER_OFFSET_FROM_ZERO = 0.0;
        public static final boolean INVERT_ABSOLUTE_ENCODER = true;

        public static final int RELATIVE_ENCODER_PORT_A = 2;
        public static final int RELATIVE_ENCODER_PORT_B = 1;
        public static final boolean INVERT_RELATIVE_ENCODER = false;

        public static final TunablePIDGains PID_GAINS =
                new TunablePIDGains("/gains/lifter", 8, 0.0, 3.5161, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
                new TunableTrapezoidalProfileGains("/gains/lifter", 1.2, 1.2, MiscConstants.TUNING_MODE);
        public static final TunableArmFFGains FF_GAINS =
                new TunableArmFFGains("/gains/lifter", 0.12116, 0.22606, 9.737, 0.15967, MiscConstants.TUNING_MODE);

        public static final int STALL_CURRENT_LIMIT = 10;
        public static final int FREE_CURRENT_LIMIT = 40;

        public static final double HOME_CURRENT = 0.3;
        public static final double HOME_VOLTAGE = -0.5;

        public static final double POSITION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);
        public static final double VELOCITY_TOLERANCE_RADIANS_SECOND = Units.degreesToRadians(10.0);
    }

    public static class ExtensionConstants {
        private ExtensionConstants() {}

        public static final int LEFT_MOTOR_PORT = 3;
        public static final int RIGHT_MOTOR_PORT = 4;

        public static final boolean INVERT_LEFT = true;
        public static final boolean INVERT_RIGHT = false;

        public static final int STALL_CURRENT_LIMIT = 10;
        public static final int FREE_CURRENT_LIMIT = 20;

        public static final double GEAR_REDUCTION = 3.0 * 5.0;

        // It's a 0.5in hex shaft
        public static final double METERS_PER_REV = Units.inchesToMeters(0.294 * 6);

        // Offset from claw to center of robot when at "0"
        public static final double X_OFFSET_METERS = Units.inchesToMeters(8.0);

        public static final double MIN_POSITION = 0.0;
        public static final double MAX_POSITION = Units.inchesToMeters(22.0);

        public static final TunablePIDGains PID_GAINS =
                new TunablePIDGains("/gains/extension", 15.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
        public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
                new TunableTrapezoidalProfileGains("/gains/extension", 0.5, 0.6, MiscConstants.TUNING_MODE);
        public static final TunableFFGains FF_GAINS =
                new TunableFFGains("/gains/extension", 0.27288, 20.188, 2.1074, MiscConstants.TUNING_MODE);

        public static final double HOME_CURRENT = 10;
        public static final double HOME_VOLTAGE = -1;

        public static final double POSITION_TOLERANCE_METERS = Units.inchesToMeters(1.0);
        public static final double VELOCITY_TOLERANCE_METERS_SECOND = Units.inchesToMeters(1.0);
    }

    public static class LEDConstants {
        public static final int PWM_PORT = 0;
        public static final int FRONT_LEFT_SIZE = 14;
        public static final int FRONT_RIGHT_SIZE = 14;
        public static final int BACK_LEFT_SIZE = 17;
        public static final int BACK_RIGHT_SIZE = 11;

        public static final int TOTAL_SIZE = FRONT_LEFT_SIZE + FRONT_RIGHT_SIZE + BACK_LEFT_SIZE + BACK_RIGHT_SIZE;
        public static final int MAX_SIZE =
                Collections.max(List.of(BACK_LEFT_SIZE, FRONT_LEFT_SIZE, BACK_RIGHT_SIZE, FRONT_RIGHT_SIZE));
    }

    public static class TeleopConstants {
        private TeleopConstants() {}

        public static final boolean OPEN_LOOP_DRIVETRAIN = true;
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

        public static final double ROBOT_SCORING_X = Grids.outerX + (MiscConstants.FULL_ROBOT_LENGTH_METERS / 2.0);

        // These are hard-coded right now because of the drift and stuff, but this will be changed once we get the
        // encoder
        public static final Pair<Rotation2d, Double> HIGH = Pair.of(Rotation2d.fromDegrees(26.0), 0.5685);
        public static final Pair<Rotation2d, Double> MID = Pair.of(Rotation2d.fromDegrees(10.08), 0.02839);
        public static final Pair<Rotation2d, Double> LOW = Pair.of(Rotation2d.fromDegrees(-72.39), 0.1903);
        public static final Pair<Rotation2d, Double> SUBSTATION = Pair.of(Rotation2d.fromDegrees(10.2), 0.2);
        public static final Pair<Rotation2d, Double> STOW =
                Pair.of(LiftConstants.MIN_ANGLE.plus(Rotation2d.fromDegrees(1.0)), Units.inchesToMeters(0.5));
        public static final Pair<Rotation2d, Double> CARRY =
                Pair.of(LiftConstants.MIN_ANGLE.plus(Rotation2d.fromDegrees(20.0)), Units.inchesToMeters(0.5));

        public static final Pose2d[] preScoreFromLocations = new Pose2d[Grids.highTranslations.length];
        public static final Pose2d[] scoreFromLocations = new Pose2d[Grids.highTranslations.length];

        static {
            for (int i = 0; i < scoreFromLocations.length; i++) {
                scoreFromLocations[i] = new Pose2d(
                        new Translation2d(ROBOT_SCORING_X, Grids.lowTranslations[i].getY()),
                        Rotation2d.fromDegrees(180.0));
                double yOffset = 0.0;
                if (i == 1) {
                    yOffset = Units.inchesToMeters(-6.0);
                } else if (i == 7) {
                    yOffset = Units.inchesToMeters(6.0);
                }
                preScoreFromLocations[i] = new Pose2d(
                        scoreFromLocations[i].getTranslation().plus(new Translation2d(0.5, yOffset)),
                        scoreFromLocations[i].getRotation());
            }
        }

        private static final double SUBSTATION_PICKUP_X = LoadingZone.doubleSubstationX
                - (MiscConstants.FULL_ROBOT_LENGTH_METERS / 2.0)
                - LiftExtensionKinematics.liftExtensionPositionToClawPosition(
                                SUBSTATION.getFirst(), SUBSTATION.getSecond())
                        .getX();
        public static final Pose2d WALL_SIDE_SUBSTATION_PICKUP = new Pose2d(
                new Translation2d(SUBSTATION_PICKUP_X, LoadingZone.leftY - Units.inchesToMeters(18.0)),
                Rotation2d.fromDegrees(0.0));
        public static final Pose2d NOT_WALL_SIDE_SUBSTATION_PICKUP = new Pose2d(
                new Translation2d(SUBSTATION_PICKUP_X, LoadingZone.rightY + Units.inchesToMeters(18.0)),
                Rotation2d.fromDegrees(0.0));

        public static final Rectangle ALLOWED_SCORING_AREA = new Rectangle(
                new Translation2d(Community.innerX, Community.rightY),
                new Translation2d(Community.chargingStationInnerX, Community.leftY));

        public static final Rectangle ALLOWED_SUBSTATION_AREA = new Rectangle(
                new Translation2d(LoadingZone.midX, LoadingZone.rightY),
                new Translation2d(LoadingZone.innerX, LoadingZone.leftY));
    }

    public static class MiscConstants {

        private MiscConstants() {}

        public static final int[] USED_CONTROLLER_PORTS = {0};
        public static final boolean TUNING_MODE = true;

        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;
        public static final int POWER_MODULE_ID = 1;
        public static final double CONFIGURATION_TIMEOUT_SECONDS = 5.0;

        public static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(3);
        // With bumpers
        public static final double FULL_ROBOT_LENGTH_METERS = Units.inchesToMeters(26.0) + (BUMPER_WIDTH_METERS * 2);
        public static final double FULL_ROBOT_WIDTH_METERS = Units.inchesToMeters(33.866) + (BUMPER_WIDTH_METERS * 2);

        public static final double LONGEST_SIDE_METERS = Math.max(FULL_ROBOT_LENGTH_METERS, FULL_ROBOT_WIDTH_METERS);
    }
}
