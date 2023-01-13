package frc.robot.subsystems.swerve;

import static frc.robot.Constants.DriveTrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.rich.ChassisSpeedsEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.RaiderMathUtils;
import frc.robot.subsystems.swerve.PhotonCameraWrapperSubsystem;

/** The subsystem containing all the swerve modules */
public class SwerveDriveSubsystem extends SubsystemBase {
    enum DriveMode {
        OPEN_LOOP,
        CLOSE_LOOP,
        CHARACTERIZATION,
        RAW_VOLTAGE
    }

    private final SwerveModule[] modules = new SwerveModule[NUM_MODULES];

    private final PhotonCameraWrapperSubsystem cameraSubsystem = new PhotonCameraWrapperSubsystem();

    private final AHRS gyro = new AHRS();

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Alert navXNotConnectedFaultAlert = new Alert(
            "navX is not connected. Field-centric drive and odometry will be negatively effected!", AlertType.ERROR);
    private final Alert navXCalibratingAlert = new Alert("navX is calibrating. Keep the robot still!", AlertType.INFO);
    private final DoubleTelemetryEntry gyroEntry = new DoubleTelemetryEntry("/drive/gyroDegrees", true);
    private final ChassisSpeedsEntry chassisSpeedsEntry =
            new ChassisSpeedsEntry("/drive/speeds", MiscConstants.TUNING_MODE);
    private final DoubleArrayTelemetryEntry odometryEntry =
            new DoubleArrayTelemetryEntry("/drive/estimatedPose", false);
    private final EventTelemetryEntry driveEventLogger = new EventTelemetryEntry("/drive/events");

    private final Field2d field2d = new Field2d();

    private SwerveModuleState[] desiredStates = new SwerveModuleState[NUM_MODULES];
    private SwerveModuleState[] nextStates = new SwerveModuleState[NUM_MODULES];
    private boolean activeSteer = true;
    private DriveMode driveMode = DriveMode.OPEN_LOOP;
    private double rawDriveVolts = 0.0;
    private double rawSteerVolts = 0.0;

    public SwerveDriveSubsystem() {
        modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
        modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
        modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
        modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);

        driveEventLogger.append("Swerve modules initialized");

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getGyroRotation(), getModulePositions(), new Pose2d());

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");

        driveTab.add("Field", field2d);
        driveTab.add(
                "Reset to Absolute",
                Commands.runOnce(this::setAllModulesToAbsolute)
                        .ignoringDisable(true)
                        .withName("Reset"));
        driveTab.addBoolean("All have been set to absolute", this::allModulesAtAbsolute);
        driveTab.add("Kill Front Left (0)", modules[0].getToggleDeadModeCommand());
        driveTab.add("Kill Front Right (1)", modules[1].getToggleDeadModeCommand());
        driveTab.add("Kill Back Left (2)", modules[2].getToggleDeadModeCommand());
        driveTab.add("Kill Back Right (3)", modules[3].getToggleDeadModeCommand());

        stopMovement();
    }

    /**
     * @return the value from the gyro. This does not get reset when resetOdometry is called. Use
     *     <code>getPose().getRotation2d()</code> for reset value. Counterclockwise is positive.
     */
    private Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw() * (INVERT_GYRO ? -1 : 1));
    }

    /** Sets the odometry perceived location to zero */
    public void zeroHeading() {
        setHeading(Rotation2d.fromDegrees(0.0));
    }

    /**
     * Set the odometry perceived location to the provided heading
     *
     * @param newHeading the provided heading
     */
    public void setHeading(Rotation2d newHeading) {
        Pose2d currentPose = getPose();

        Pose2d newPose = new Pose2d(currentPose.getTranslation(), newHeading);

        resetOdometry(newPose);
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    /**
     * Set the current perceived location of the robot to the provided pose
     *
     * @param pose2d the provided pose
     */
    public void resetOdometry(Pose2d pose2d) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose2d);

        driveEventLogger.append("Estimator reset");
    }

    /**
     * @return the estimated position of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }



    public ChassisSpeeds getCurrentChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getActualStates());
    }

    /**
     * Set the desired speed of the robot. Chassis speeds are always robot centric but can be created
     * from field centric values through {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double,
     * double, Rotation2d)}
     *
     * @param chassisSpeeds the desired chassis speeds
     * @param openLoop if true then velocity will be handled exclusivity with feedforward (mostly used
     *     for teleop). If false a PIDF will be used (mostly used for auto)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean openLoop) {
        setRawStates(true, openLoop, KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Set the desired speed of the robot. Chassis speeds are always robot centric but can be created
     * from field centric values through {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double,
     * double, Rotation2d)}. This will correct for skew.
     *
     * @param chassisSpeeds the desired chassis speeds
     * @param nextChassisSpeeds the speeds that will be next, used for calculating acceleration
     * @param openLoop if true then velocity will be handled exclusivity with feedforward (mostly used
     *     for teleop). If false a PIDF will be used (mostly used for auto)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, ChassisSpeeds nextChassisSpeeds, boolean openLoop) {
        setRawStates(
                true,
                openLoop,
                KINEMATICS.toSwerveModuleStates(chassisSpeeds),
                KINEMATICS.toSwerveModuleStates(nextChassisSpeeds));
    }

    /**
     * Sets the desired swerve drive states for the modules. This method also takes a copy of the
     * states, so they will not be changed. Assumes zero acceleration.
     *
     * @param activeSteer if false will not actively power the steer motor
     * @param openLoop if true then velocity will be handled exclusivity with feedforward (for teleop
     *     mostly). If false a PIDF will be used (for auto)
     * @param states the desired states... Ordered front left, front right, back left, back right
     */
    public void setRawStates(boolean activeSteer, boolean openLoop, SwerveModuleState[] states) {
        setRawStates(activeSteer, openLoop, states, states);
    }

    /**
     * Sets the desired swerve drive states for the modules. This method also takes a copy of the
     * states, so they will not be changed
     *
     * @param activeSteer if false will not actively power the steer motor
     * @param openLoop if true then velocity will be handled exclusivity with feedforward (for teleop
     *     mostly). If false a PIDF will be used (for auto)
     * @param desiredStates the desired states... Ordered front left, front right, back left, back right
     * @param nextStates the states that will be used for the acceleration ff
     */
    public void setRawStates(
            boolean activeSteer, boolean openLoop, SwerveModuleState[] desiredStates, SwerveModuleState[] nextStates) {
        if (desiredStates.length != modules.length) {
            throw new IllegalArgumentException("You must provide desiredStates for all modules");
        }

        driveMode = openLoop ? DriveMode.OPEN_LOOP : DriveMode.CLOSE_LOOP;
        this.activeSteer = activeSteer;

        this.desiredStates = RaiderMathUtils.copySwerveStateArray(desiredStates);
        this.nextStates = RaiderMathUtils.copySwerveStateArray(nextStates);
    }

    /**
     * Set the voltage directly for the motors.
     *
     * @param driveVolts the desired drive voltage
     * @param steerVolts the desired steer voltage
     */
    public void setRawVolts(double driveVolts, double steerVolts) {
        driveMode = DriveMode.RAW_VOLTAGE;

        this.rawDriveVolts = driveVolts;
        this.rawSteerVolts = steerVolts;
    }

    /** Sets each module velocity to zero and desired angle to what it currently is */
    public void stopMovement() {
        SwerveModuleState[] newStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            newStates[i] = new SwerveModuleState(0.0, modules[i].getActualState().angle);
        }
        setRawStates(false, true, newStates);
    }

    /**
     * Set the module to characterization mode with the provided voltage
     *
     * @param voltage the voltage to apply to the drive motor
     */
    public void setCharacterizationVoltage(double voltage) {
        driveMode = DriveMode.CHARACTERIZATION;
        rawDriveVolts = voltage;
    }

    public double getAverageDriveVelocityMetersSecond() {
        SwerveModuleState[] actualStates = getActualStates();
        double sum = 0.0;
        for (SwerveModuleState state : actualStates) {
            sum += state.speedMetersPerSecond;
        }

        return sum / actualStates.length;
    }

    public double getAverageDrivePositionMeters() {
        SwerveModulePosition[] positions = getModulePositions();
        double sum = 0.0;
        for (SwerveModulePosition position : positions) {
            sum += position.distanceMeters;
        }

        return sum / positions.length;
    }

    public void resetModuleEncoderPositions() {
        for (SwerveModule module : modules) {
            module.resetDriveMotorPosition();
        }
    }

    public void setAllModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.resetSteerToAbsolute();
        }
    }

    private boolean allModulesAtAbsolute() {
        boolean allSet = true;
        for (SwerveModule module : modules) {
            allSet &= module.isSetToAbsolute();
        }
        return allSet;
    }

    private SwerveModuleState[] getActualStates() {
        SwerveModuleState[] actualStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            actualStates[i] = modules[i].getActualState();
        }
        return actualStates;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] actualPositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            actualPositions[i] = modules[i].getActualPosition();
        }

        return actualPositions;
    }

    @Override
    public void periodic() {
        Robot.startWNode("SwerveDriveSubsystem#periodic");
        Robot.startWNode("setDesiredStates");
        switch (driveMode) {
            case OPEN_LOOP, CLOSE_LOOP -> {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_SECOND);
                SwerveDriveKinematics.desaturateWheelSpeeds(nextStates, MAX_VELOCITY_METERS_SECOND);
                for (int i = 0; i < modules.length; i++) {
                    modules[i].setDesiredState(
                            desiredStates[i], nextStates[i], activeSteer, driveMode == DriveMode.OPEN_LOOP);
                }
            }
            case RAW_VOLTAGE -> {
                for (SwerveModule module : modules) {
                    module.setRawVoltage(rawDriveVolts, rawSteerVolts);
                }
            }
            case CHARACTERIZATION -> {
                for (SwerveModule module : modules) {
                    module.setCharacterizationVoltage(rawDriveVolts);
                }
            }
        }
        Robot.endWNode();

        Robot.startWNode("odometry");
        poseEstimator.update(getGyroRotation(), getModulePositions());
        Robot.endWNode();

        Robot.startWNode("logging");
        logValues();
        Robot.endWNode();
        Robot.endWNode();

        Pair<Pose2d, Double> timeStampCameraPose = cameraSubsystem.getVisionPose(poseEstimator.getEstimatedPosition());
        poseEstimator.addVisionMeasurement(timeStampCameraPose.getFirst(), timeStampCameraPose.getSecond());

    }

    double[] estimatedPoseLoggingArray = new double[3];

    private void logValues() {
        gyroEntry.append(getGyroRotation().getDegrees());

        Pose2d estimatedPose = getPose();
        estimatedPoseLoggingArray[0] = estimatedPose.getX();
        estimatedPoseLoggingArray[1] = estimatedPose.getY();
        estimatedPoseLoggingArray[2] = estimatedPose.getRotation().getRadians();
        odometryEntry.append(estimatedPoseLoggingArray);

        chassisSpeedsEntry.append(getCurrentChassisSpeeds());

        field2d.setRobotPose(estimatedPose);
        for (int i = 0; i < modules.length; i++) {
            field2d.getObject("module " + i)
                    .setPose(estimatedPose.plus(
                            new Transform2d(MODULE_TRANSLATIONS[i], modules[i].getActualState().angle)));
        }

        for (SwerveModule module : modules) {
            module.logValues();
        }

        navXNotConnectedFaultAlert.set(!gyro.isConnected());
        navXCalibratingAlert.set(gyro.isCalibrating());
    }

    public Field2d getField2d() {
        return field2d;
    }
}
