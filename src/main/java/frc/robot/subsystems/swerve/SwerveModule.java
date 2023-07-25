package frc.robot.subsystems.swerve;

import static frc.robot.utils.RaiderUtils.checkCTREError;
import static frc.robot.utils.RaiderUtils.checkRevError;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;
import frc.robot.utils.RaiderMathUtils;
import frc.robot.utils.SwerveModuleConfiguration;

public class SwerveModule {
    private enum SwerveModuleControlMode {
        NORMAL(1),
        CHARACTERIZATION(2),
        RAW_VOLTAGE(3),
        DEAD_MODE(4);

        final int logValue;

        SwerveModuleControlMode(int logValue) {
            this.logValue = logValue;
        }
    }

    private static final int CAN_TIMEOUT_MS = 250;

    private static final int CANCODER_PERIOD_MS = 50;

    private static int instances = 0;

    private final int instanceId;
    private final DoubleTelemetryEntry driveVelocitySetpointEntry;
    private final DoubleTelemetryEntry nextDriveVelocitySetpointEntry;
    private final BooleanTelemetryEntry openLoopEntry;
    private final DoubleTelemetryEntry steerPositionGoalEntry;
    private final DoubleTelemetryEntry feedForwardOutputEntry;
    private final BooleanTelemetryEntry activeSteerEntry;
    private final DoubleTelemetryEntry absoluteHeadingEntry;
    private final BooleanTelemetryEntry setToAbsoluteEntry;
    // 1 is regular, 2 is characterization, 3 is raw voltage, 4 is dead mode
    private final IntegerTelemetryEntry controlModeEntry;
    private final EventTelemetryEntry moduleEventEntry;

    private final Alert notSetToAbsoluteAlert;
    private final Alert steerEncoderFaultAlert;
    private final Alert steerMotorFaultAlert;
    private final Alert driveMotorFaultAlert;
    private final Alert inDeadModeAlert;

    private final TelemetryTalonFX driveMotor;
    private final TelemetryCANSparkMax steerMotor;
    private final RelativeEncoder absoluteSteerEncoder;

    private final double driveMotorConversionFactorPosition;
    private final double driveMotorConversionFactorVelocity;
    private final double steerMotorConversionFactorPosition;
    private final double steerMotorConversionFactorVelocity;
    private final double nominalVoltage;
    private final double openLoopMaxSpeed;
    private final double steerEncoderOffsetRadians;

    private final TunablePIDGains driveVelocityPIDGains;
    private final TunableFFGains driveVelocityFFGains;
    private final TunablePIDGains steerPositionPIDGains;
    private SimpleMotorFeedforward driveMotorFF;

    private final SparkMaxPIDController steerController;

    private boolean setToAbsolute = false;
    private boolean isDeadMode = false;
    private double lastMoveTime = 0.0;
    private double lastAbsoluteResetTime = 0.0;

    /** Constructs a new Swerve Module using the given config */
    public SwerveModule(SwerveModuleConfiguration config, boolean tuningMode) {
        instanceId = instances++;

        // Initialize all telemetry entries
        String tableName = "/drive/modules/" + instanceId + "/";
        driveVelocitySetpointEntry = new DoubleTelemetryEntry(tableName + "driveVelocitySetpoint", true);
        nextDriveVelocitySetpointEntry = new DoubleTelemetryEntry(tableName + "nextDriveVelocitySetpoint", tuningMode);
        openLoopEntry = new BooleanTelemetryEntry(tableName + "openLoop", tuningMode);
        feedForwardOutputEntry = new DoubleTelemetryEntry(tableName + "feedforwardOutput", tuningMode);
        steerPositionGoalEntry = new DoubleTelemetryEntry(tableName + "steerPositionGoal", true);
        activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", tuningMode);
        absoluteHeadingEntry = new DoubleTelemetryEntry(tableName + "absoluteHeading", tuningMode);
        setToAbsoluteEntry = new BooleanTelemetryEntry(tableName + "setToAbsolute", true);
        controlModeEntry = new IntegerTelemetryEntry(tableName + "controlMode", false);
        moduleEventEntry = new EventTelemetryEntry(tableName + "events");

        String alertPrefix = "Module " + instanceId + ": ";
        notSetToAbsoluteAlert = new Alert(alertPrefix + "Steer is not reset to absolute position", AlertType.ERROR);
        steerEncoderFaultAlert = new Alert(alertPrefix + "Steer encoder had a fault initializing", AlertType.ERROR);
        steerMotorFaultAlert = new Alert(alertPrefix + "Steer motor had a fault initializing", AlertType.ERROR);
        driveMotorFaultAlert = new Alert(alertPrefix + "Drive motor had a fault initializing", AlertType.ERROR);
        inDeadModeAlert = new Alert(alertPrefix + "In dead mode", AlertType.WARNING);

        this.driveMotorConversionFactorPosition = (config.sharedConfiguration().wheelDiameterMeters() * Math.PI)
                / (config.sharedConfiguration().driveGearRatio() * 2048);
        this.driveMotorConversionFactorVelocity = driveMotorConversionFactorPosition * 10.0;
        this.steerMotorConversionFactorPosition =
                (Math.PI * 2) / (config.sharedConfiguration().steerGearRatio() * 2048);
        this.steerMotorConversionFactorVelocity = steerMotorConversionFactorPosition * 10.0;

        this.driveVelocityPIDGains = config.sharedConfiguration().driveVelocityPIDGains();
        this.driveVelocityFFGains = config.sharedConfiguration().driveVelocityFFGains();

        this.steerPositionPIDGains = config.sharedConfiguration().steerPositionPIDGains();

        // Drive motor

        this.driveMotor = new TelemetryTalonFX(
                config.driveMotorPort(),
                tableName + "driveMotor",
                config.sharedConfiguration().canBus(),
                tuningMode);
        configDriveMotor(config);

        // Steer encoder

        configSteerEncoder(config);

        // Steer motor
        this.steerMotor = new TelemetryCANSparkMax(
                config.steerMotorPort(), CANSparkMax.MotorType.kBrushless, tableName + "steerMotor", tuningMode);
        //        configSteerMotor(config);
        absoluteSteerEncoder = steerMotor.getEncoder();
        this.nominalVoltage = config.sharedConfiguration().nominalVoltage();
        this.openLoopMaxSpeed = config.sharedConfiguration().openLoopMaxSpeed();
        this.steerEncoderOffsetRadians = config.offsetRadians();

        this.driveMotorFF = driveVelocityFFGains.createFeedforward();

        resetSteerToAbsolute(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
    }

    private void configDriveMotor(SwerveModuleConfiguration config) {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing;
        do {
            faultInitializing = checkCTREError(driveMotor.configFactoryDefault(CAN_TIMEOUT_MS));

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            applyCommonMotorConfiguration(motorConfiguration, config);

            motorConfiguration.supplyCurrLimit.currentLimit =
                    config.sharedConfiguration().driveContinuousCurrentLimit();
            motorConfiguration.supplyCurrLimit.triggerThresholdCurrent =
                    config.sharedConfiguration().drivePeakCurrentLimit();
            motorConfiguration.supplyCurrLimit.triggerThresholdTime =
                    config.sharedConfiguration().drivePeakCurrentDurationSeconds();

            motorConfiguration.closedloopRamp = config.sharedConfiguration().driveClosedLoopRamp();
            motorConfiguration.openloopRamp = config.sharedConfiguration().driveOpenLoopRamp();

            config.sharedConfiguration().driveVelocityPIDGains().setSlot(motorConfiguration.slot0);

            faultInitializing |= checkCTREError(driveMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS));

            driveMotor.setInverted(
                    config.driveMotorInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);

            faultInitializing |= checkCTREError(
                    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS));

            driveMotor.setSensorPhase(false);

            driveMotor.setNeutralMode(NeutralMode.Brake);

            driveMotor.setLoggingPositionConversionFactor(driveMotorConversionFactorPosition);
            driveMotor.setLoggingVelocityConversionFactor(driveMotorConversionFactorVelocity);

            // Clear the reset of it starting up
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        driveMotor.hasResetOccurred();
        driveMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));
    }
    // TODO: same config??

    private void configSteerMotor(SwerveModuleConfiguration config) {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing;

        do {
            faultInitializing = checkRevError(steerMotor.restoreFactoryDefaults());

            //             motorConfiguration = new TelemetryCANSparkMax.();
            //            applyCommonMotorConfiguration(motorConfiguration, config);

            //            motorConfiguration.sharedConfiguration().steerContinuousCurrentLimit() =
            //                    config.sharedConfiguration().steerContinuousCurrentLimit();

            steerMotor.setSmartCurrentLimit((int) config.sharedConfiguration().steerContinuousCurrentLimit(), (int)
                    config.sharedConfiguration().steerPeakCurrentLimit());

            //            motorConfiguration.supplyCurrLimit.triggerThresholdCurrent =
            //                    config.sharedConfiguration().steerPeakCurrentLimit();

            //            motorConfiguration.supplyCurrLimit.triggerThresholdTime =
            //                    config.sharedConfiguration().steerPeakCurrentDurationSeconds();

            steerMotor.setClosedLoopRampRate(config.sharedConfiguration().steerClosedLoopRamp());

            //            motorConfiguration.closedloopRamp = config.sharedConfiguration().steerClosedLoopRamp();

            //            config.sharedConfiguration().steerPositionPIDGains().setSlot(motorConfiguration.slot0);
            //            motorConfiguration.slot0.allowableClosedloopError =
            //                    config.sharedConfiguration().allowableSteerErrorRadians() /
            // steerMotorConversionFactorPosition;

            // TODO: I have no clue if this works
            steerController.setOutputRange(
                    config.sharedConfiguration().nominalVoltage(),
                    config.sharedConfiguration().maxSteerVoltage());

            //            motorConfiguration.slot0.closedLoopPeakOutput = config.sharedConfiguration()
            //                            .maxSteerVoltage()
            //                    / config.sharedConfiguration().nominalVoltage();

            //            faultInitializing |= checkCTREError(steerMotor.set(motorConfiguration, CAN_TIMEOUT_MS));

            faultInitializing |= checkRevError(steerMotor.setCANTimeout(CAN_TIMEOUT_MS));

            //            absoluteSteerEncoder.setInverted(
            //                    config.steerMotorInverted() == true ? absoluteSteerEncoder.setInverted(true) :
            // absoluteSteerEncoder.setInverted(false));

            if (config.steerMotorInverted()) {
                absoluteSteerEncoder.setInverted(true);
            } else {
                absoluteSteerEncoder.setInverted(false);
            }

            faultInitializing |= checkRevError(steerController.setFeedbackDevice(absoluteSteerEncoder));

            // Because + on motor is clockwise, and we want + on encoder to be
            // counter-clockwise we have to set the sensor phase

            // TODO: I dont think we need this for SM
            //            steerMotor.setSensorPhase(false);

            steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

            absoluteSteerEncoder.setPositionConversionFactor(steerMotorConversionFactorPosition);
            absoluteSteerEncoder.setVelocityConversionFactor(steerMotorConversionFactorVelocity);
            //            steerMotor.setLoggingPositionConversionFactor(steerMotorConversionFactorPosition);
            //            steerMotor.setLoggingVelocityConversionFactor(steerMotorConversionFactorVelocity);

        } while (faultInitializing && configTimeout.hasNotTimedOut());

        // Clear the reset of it starting up
        //        steerMotor.hasResetOccurred();

        steerMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void applyCommonMotorConfiguration(
            TalonFXConfiguration motorConfiguration, SwerveModuleConfiguration config) {
        motorConfiguration.supplyCurrLimit.enable = true;
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration().nominalVoltage();
    }

    // TODO: configure a steer motor for spark max???

    private void configSteerEncoder(SwerveModuleConfiguration config) {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        //        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();
        //
        //        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        //        encoderConfiguration.sensorDirection = config.steerEncoderInverted();
        //        encoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        boolean faultInitializing;
        do {
            faultInitializing =
                    //                    REVLibError(absoluteSteerEncoder.(encoderConfiguration, CAN_TIMEOUT_MS));

                    // Because we are only reading this at the beginning we do not have to update it
                    // often
                    // TODO: could be the wrong time period
                    checkRevError(absoluteSteerEncoder.setMeasurementPeriod(CANCODER_PERIOD_MS));
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        steerEncoderFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer encoder initialized" + (faultInitializing ? " with faults" : ""));
    }

    /**
     * Reports error to event log and driver station. Prepends module details to DS report.
     *
     * @param message The message to report
     */
    private void reportError(String message) {
        moduleEventEntry.append(message);
        DriverStation.reportError(String.format("Module %d: %s", instanceId, message), false);
    }

    //    private void checkForSteerMotorReset() {
    //        // Steer motor lost power
    //        if (RobotBase.isReal() && steerMotor.getL()) {
    //            reportError("Steer motor reset occurred");
    //            setToAbsolute = false;
    //            resetSteerToAbsolute();
    //        }
    //    }

    private void checkForDriveMotorReset() {
        if (RobotBase.isReal() && driveMotor.hasResetOccurred()) {
            reportError("Drive motor reset occurred");
        }
    }

    /**
     * Resets the integrated encoder on the Steer motor to the absolute position of the CANCoder. Trys
     * only once, and if it fails, it will not try again until
     */
    public void resetSteerToAbsolute() {
        resetSteerToAbsolute(0.0);
    }

    /**
     * Resets the integrated encoder on the Steer motor to the absolute position of the CANCoder
     *
     * @param timeout The timeout in seconds to wait.
     */
    public void resetSteerToAbsolute(double timeout) {
        double startTime = Timer.getFPGATimestamp();
        setToAbsolute = false;

        double absolutePosition;
        boolean gotAbsolutePosition = false;
        do {
            absolutePosition = getAbsoluteRadians();
            if (steerMotor.getLastError() == null || timeout == 0.0) {
                // TODO: IDK if this works

                gotAbsolutePosition = true;
            }
            if (gotAbsolutePosition) {

                REVLibError settingPositionError =
                        absoluteSteerEncoder.setPosition(absolutePosition / steerMotorConversionFactorPosition);
                // If no error
                if (!checkRevError(settingPositionError) || timeout == 0.0) {
                    setToAbsolute = true;
                }
            }
        } while (!setToAbsolute && timeout != 0.0 && Timer.getFPGATimestamp() - startTime < timeout);

        if (!setToAbsolute) {
            reportError("CANCoder/TalonFX timed out while trying to get absolute position");
        } else {
            lastAbsoluteResetTime = Timer.getFPGATimestamp();
            moduleEventEntry.append("Reset steer motor encoder to position: " + absolutePosition);
        }

        notSetToAbsoluteAlert.set(!setToAbsolute);
    }

    public boolean isSetToAbsolute() {
        return setToAbsolute;
    }

    private double getAbsoluteRadians() {
        return MathUtil.angleModulus(
                Units.degreesToRadians(absoluteSteerEncoder.getPosition()) + steerEncoderOffsetRadians);
    }

    private double getSteerAngleRadiansNoWrap() {
        return absoluteSteerEncoder.getPosition() * steerMotorConversionFactorPosition;
    }

    /**
     * @return the rotation of the wheel
     */
    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(MathUtil.angleModulus(getSteerAngleRadiansNoWrap()));
    }

    private double getSteerVelocityRadiansPerSecond() {
        return absoluteSteerEncoder.getVelocity() * steerMotorConversionFactorVelocity;
    }

    private double getDriveVelocityMetersPerSecond() {
        return driveMotor.getSelectedSensorVelocity() * driveMotorConversionFactorVelocity;
    }

    private double getDriveMotorPositionMeters() {
        return driveMotor.getSelectedSensorPosition() * driveMotorConversionFactorPosition;
    }

    public void setDeadModule(boolean isDeadMode) {
        this.isDeadMode = isDeadMode;
        inDeadModeAlert.set(isDeadMode);

        steerMotor.setIdleMode(isDeadMode ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        driveMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void resetDriveMotorPosition() {
        driveMotor.setSelectedSensorPosition(0.0);
    }

    public CommandBase getToggleDeadModeCommand() {
        return Commands.runOnce(() -> setDeadModule(!isDeadMode))
                .ignoringDisable(true)
                .withName("ToggleDeadMode");
    }

    /**
     * @return the current state of the modules as reported from the encoders
     */
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getSteerAngle());
    }

    public SwerveModulePosition getActualPosition() {
        return new SwerveModulePosition(getDriveMotorPositionMeters(), getSteerAngle());
    }

    public double getActualDriveVoltage() {
        return driveMotor.getMotorOutputVoltage();
    }

    /**
     * Set the desired state for this swerve module
     *
     * @param state the desired state
     * @param activeSteer if steer should be active
     * @param openLoop if velocity control should be feed forward only. False if to use PIDF for
     *     velocity control.
     */
    public void setDesiredState(SwerveModuleState state, boolean activeSteer, boolean openLoop) {
        Robot.startWNode("SwerveModule[" + instanceId + "]#setDesiredState");
        Robot.startWNode("checkForResetAndGains");
        //        absoluteSteerEncoder.;
        checkForDriveMotorReset();
        checkAndUpdateGains();
        Robot.endWNode();

        if (isDeadMode) {
            controlModeEntry.append(SwerveModuleControlMode.DEAD_MODE.logValue);
            return;
        }
        controlModeEntry.append(SwerveModuleControlMode.NORMAL.logValue);

        double currentTime = Timer.getFPGATimestamp();

        if (state.speedMetersPerSecond != 0.0 || activeSteer) {
            lastMoveTime = currentTime;
        }

        state = SwerveModuleState.optimize(state, getSteerAngle());

        Robot.startWNode("setDriveState");
        setDriveReference(state.speedMetersPerSecond, openLoop);
        Robot.endWNode();

        Robot.startWNode("setSteerState");
        setSteerReference(state.angle.getRadians(), activeSteer);
        Robot.endWNode();

        if (shouldResetToAbsolute()) {
            Robot.startWNode("resetSteerToAbsolute");
            resetSteerToAbsolute();
            Robot.endWNode();
        }
        Robot.endWNode();
    }

    public void setCharacterizationVoltage(double voltage) {
        controlModeEntry.append(SwerveModuleControlMode.CHARACTERIZATION.logValue);

        setSteerReference(0.0, true);

        // Divide by the value our voltage compensation is set as
        driveMotor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
    }

    public void setRawVoltage(double driveVolts, double steerVolts) {
        controlModeEntry.append(SwerveModuleControlMode.RAW_VOLTAGE.logValue);

        driveMotor.set(TalonFXControlMode.PercentOutput, driveVolts / nominalVoltage);
        steerMotor.setVoltage(steerVolts / nominalVoltage);
    }

    private void setSteerReference(double targetAngleRadians, boolean activeSteer) {
        activeSteerEntry.append(activeSteer);
        steerPositionGoalEntry.append(targetAngleRadians);

        if (activeSteer) {
            absoluteSteerEncoder.setPosition(
                    RaiderMathUtils.calculateContinuousInputSetpoint(getSteerAngleRadiansNoWrap(), targetAngleRadians)
                            / steerMotorConversionFactorPosition);
        } else {
            steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            // TODO: IDK how this works
        }
    }

    private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
        driveVelocitySetpointEntry.append(targetVelocityMetersPerSecond);
        openLoopEntry.append(openLoop);

        if (openLoop) {
            driveMotor.set(TalonFXControlMode.PercentOutput, targetVelocityMetersPerSecond / openLoopMaxSpeed);
        } else {
            double feedforwardValuePercent = driveMotorFF.calculate(targetVelocityMetersPerSecond) / nominalVoltage;
            feedForwardOutputEntry.append(feedforwardValuePercent);
            driveMotor.set(
                    TalonFXControlMode.Velocity,
                    targetVelocityMetersPerSecond / driveMotorConversionFactorVelocity,
                    DemandType.ArbitraryFeedForward,
                    feedforwardValuePercent);
        }
    }

    private void checkAndUpdateGains() {
        if (driveVelocityPIDGains.hasChanged() || driveVelocityFFGains.hasChanged()) {
            SlotConfiguration newSlotConfig = new SlotConfiguration();
            driveVelocityPIDGains.setSlot(newSlotConfig);
            driveMotor.configureSlot(newSlotConfig);
            driveMotorFF = driveVelocityFFGains.createFeedforward();

            moduleEventEntry.append("Updated drive gains due to value change");
        }

        if (steerPositionPIDGains.hasChanged()) {
            SlotConfiguration newSlotConfig = new SlotConfiguration();
            steerPositionPIDGains.setSlot(newSlotConfig);
            steerController = steerMotor.getPIDController();
            steerController.setP(newSlotConfig.kP);
            steerController.setD(newSlotConfig.kD);
            steerController.setI(newSlotConfig.kI);

            moduleEventEntry.append("Updated steer gains due to value change");
        }
    }

    private boolean shouldResetToAbsolute() {
        double currentTime = Timer.getFPGATimestamp();
        // If we have not reset in 5 seconds, been still for 1.5 seconds and our steer
        // velocity is less than half a degree per second (could happen if we are being
        // pushed), reset to absolute
        return DriverStation.isDisabled()
                && currentTime - lastAbsoluteResetTime > 5.0
                && currentTime - lastMoveTime > 1.5
                && Math.abs(absoluteSteerEncoder.getVelocity()) < 0.5
                && Math.abs(absoluteSteerEncoder.getVelocity() * steerMotorConversionFactorVelocity)
                        < Units.degreesToRadians(0.5);
    }

    /** Log all telemetry values. Should be called (only) in subsystem periodic */
    public void logValues() {
        driveMotor.logValues();
        steerMotor.logValues();
        absoluteHeadingEntry.append(getAbsoluteRadians());
        setToAbsoluteEntry.append(setToAbsolute);
    }
}
