package frc.robot.subsystems.swerve;

import static frc.robot.Constants.DriveTrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.rich.ChassisSpeedsEntry;
import frc.robot.telemetry.types.rich.Pose2dEntry;
import frc.robot.telemetry.types.rich.SwerveModuleStateArrayEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.RaiderCommands;
import frc.robot.utils.RaiderMathUtils;
import java.util.List;
import java.util.function.Function;
import org.photonvision.EstimatedRobotPose;

/** The subsystem containing all the swerve modules */
public class SwerveDriveSubsystem extends SubsystemBase {

    enum DriveMode {
        OPEN_LOOP,
        CLOSE_LOOP,
        CHARACTERIZATION,
        RAW_VOLTAGE
    }

    private final SwerveModule[] modules = new SwerveModule[NUM_MODULES];

    private final Function<Pose2d, List<EstimatedRobotPose>> cameraPoseDataSupplier;

    private final AHRS gyro = new AHRS();

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Alert navXNotConnectedFaultAlert = new Alert(
            "navX is not connected. Field-centric drive and odometry will be negatively effected!", AlertType.ERROR);
    private final Alert navXCalibratingAlert = new Alert("navX is calibrating. Keep the robot still!", AlertType.INFO);
    private final BooleanTelemetryEntry allModulesAtAbsoluteZeroEntry =
            new BooleanTelemetryEntry("/drive/allModulesAtAbsoluteZero", true);
    private final DoubleTelemetryEntry gyroEntry = new DoubleTelemetryEntry("/drive/gyroDegrees", true);
    private final ChassisSpeedsEntry chassisSpeedsEntry =
            new ChassisSpeedsEntry("/drive/speeds", MiscConstants.TUNING_MODE);
    private final ChassisSpeedsEntry desiredSpeedsEntry =
            new ChassisSpeedsEntry("/drive/desiredSpeeds", MiscConstants.TUNING_MODE);
    private final Pose2dEntry odometryEntry = new Pose2dEntry("/drive/estimatedPose", MiscConstants.TUNING_MODE);
    private final SwerveModuleStateArrayEntry advantageScopeSwerveDesiredStates =
            new SwerveModuleStateArrayEntry("/drive/desiredStates", MiscConstants.TUNING_MODE);
    private final SwerveModuleStateArrayEntry advantageScopeSwerveActualStates =
            new SwerveModuleStateArrayEntry("/drive/actualStates", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry rollEntry = new DoubleTelemetryEntry("/drive/roll", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry rollVelocityEntry =
            new DoubleTelemetryEntry("/drive/rollVelocity", MiscConstants.TUNING_MODE);
    private final EventTelemetryEntry driveEventLogger = new EventTelemetryEntry("/drive/events");

    private final Field2d field2d = new Field2d();

    private SwerveModuleState[] desiredStates = new SwerveModuleState[NUM_MODULES];
    private boolean activeSteer = true;
    private DriveMode driveMode = DriveMode.OPEN_LOOP;
    private double rawDriveVolts = 0.0;
    private double rawSteerVolts = 0.0;

    private double[] velocityBuffer = new double[5];
    private int velocityBufferI = 0;
    private double rollVelocity = 0.0;

    public SwerveDriveSubsystem(Function<Pose2d, List<EstimatedRobotPose>> cameraPoseDataSupplier) {
        this.cameraPoseDataSupplier = cameraPoseDataSupplier;

        modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
        modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
        modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
        modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);

        driveEventLogger.append("Swerve modules initialized");

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getGyroRotation(), getModulePositions(), new Pose2d());

        SendableTelemetryManager.getInstance().addSendable("/drive/Field", field2d);
        SendableTelemetryManager.getInstance()
                .addSendable(
                        "/drive/ResetAllModulesToAbsoluteCommand",
                        RaiderCommands.runOnceAllowDisable(this::setAllModulesToAbsolute)
                                .withName("Reset"));
        SendableTelemetryManager.getInstance()
                .addSendable("/drive/KillFrontLeft", modules[0].getToggleDeadModeCommand());
        SendableTelemetryManager.getInstance()
                .addSendable("/drive/KillFrontRight", modules[1].getToggleDeadModeCommand());
        SendableTelemetryManager.getInstance()
                .addSendable("/drive/KillBackLeft", modules[2].getToggleDeadModeCommand());
        SendableTelemetryManager.getInstance()
                .addSendable("/drive/KillBackRight", modules[3].getToggleDeadModeCommand());

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

    double pitchOffsetDegrees = 0.0;

    public double getPitchRadians() {
        return Units.degreesToRadians(gyro.getPitch() - pitchOffsetDegrees);
    }

    public void resetPitch() {
        pitchOffsetDegrees = gyro.getPitch();
    }

    double rollOffsetDegrees = 0.0;

    public double getRollRadians() {
        return Units.degreesToRadians(gyro.getRoll() - rollOffsetDegrees);
    }

    public void resetRoll() {
        rollOffsetDegrees = gyro.getRoll();
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
        desiredSpeedsEntry.append(chassisSpeeds);

        setRawStates(
                true, openLoop, KINEMATICS.toSwerveModuleStates(RaiderMathUtils.correctForSwerveSkew(chassisSpeeds)));
    }

    /**
     * Sets the desired swerve drive states for the modules. This method also takes a copy of the
     * states, so they will not be changed. Assumes zero acceleration.
     *
     * @param activeSteer if false will not actively power the steer motor
     * @param openLoop if true then velocity will be handled exclusivity with feedforward (for teleop
     *     mostly). If false a PIDF will be used (for auto)
     * @param desiredStates the desired states... Ordered front left, front right, back left, back right
     */
    public void setRawStates(boolean activeSteer, boolean openLoop, SwerveModuleState[] desiredStates) {
        if (desiredStates.length != modules.length) {
            throw new IllegalArgumentException("You must provide desiredStates for all modules");
        }

        driveMode = openLoop ? DriveMode.OPEN_LOOP : DriveMode.CLOSE_LOOP;
        this.activeSteer = activeSteer;

        this.desiredStates = RaiderMathUtils.copySwerveStateArray(desiredStates);
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

    public double[] getActualDriveVoltages() {
        double[] voltages = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            voltages[i] = modules[i].getActualDriveVoltage();
        }
        return voltages;
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

    public double getFieldCentricRoll() {
        Rotation2d yaw = getPose().getRotation();
        return yaw.getSin() * getPitchRadians() + yaw.getCos() * getRollRadians();
    }

    public double getFieldCentricRollVelocity() {
        return rollVelocity;
    }

    /**
     * Should only be used for characterization
     * @return the angle in radians
     */
    public double getRawGyroAngle() {
        return Units.degreesToRadians(gyro.getAngle());
    }

    /**
     * Should only be used for characterization
     * @return the angle rate in radians/second
     */
    public double getRawGyroRate() {
        return Units.degreesToRadians(gyro.getRate());
    }

    public SwerveModuleState[] getActualStates() {
        SwerveModuleState[] actualStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            actualStates[i] = modules[i].getActualState();
        }
        return actualStates;
    }

    public SwerveModulePosition[] getModulePositions() {
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
                for (int i = 0; i < modules.length; i++) {
                    modules[i].setDesiredState(desiredStates[i], activeSteer, driveMode == DriveMode.OPEN_LOOP);
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
        velocityBuffer[velocityBufferI] = getFieldCentricRoll();
        rollVelocity =
                (velocityBuffer[velocityBufferI] - velocityBuffer[Math.abs((velocityBufferI - 5) % 5)]) / (0.02 * 5);
        velocityBufferI++;
        velocityBufferI %= 5;

        List<EstimatedRobotPose> estimatedRobotPoses = cameraPoseDataSupplier.apply(getPose());
        for (EstimatedRobotPose estimatedRobotPose : estimatedRobotPoses) {
            if (!DriverStation.isAutonomousEnabled()) {
                poseEstimator.addVisionMeasurement(
                        estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
        }
        poseEstimator.update(getGyroRotation(), getModulePositions());
        Robot.endWNode();

        Robot.startWNode("logValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        allModulesAtAbsoluteZeroEntry.append(allModulesAtAbsolute());

        gyroEntry.append(getGyroRotation().getDegrees());

        Pose2d estimatedPose = getPose();
        odometryEntry.append(estimatedPose);

        chassisSpeedsEntry.append(getCurrentChassisSpeeds());

        rollEntry.append(getFieldCentricRoll());
        rollVelocityEntry.append(getFieldCentricRollVelocity());

        field2d.setRobotPose(estimatedPose);

        for (SwerveModule module : modules) {
            module.logValues();
        }

        advantageScopeSwerveDesiredStates.append(desiredStates);
        advantageScopeSwerveActualStates.append(getActualStates());

        navXNotConnectedFaultAlert.set(!gyro.isConnected());
        navXCalibratingAlert.set(gyro.isCalibrating());
    }

    public Field2d getField2d() {
        return field2d;
    }
}
