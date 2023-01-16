package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.telemetry.tunable.TunableFFGains;
import frc.robot.telemetry.tunable.TunablePIDGains;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
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
    private static final double INITIAL_TIMEOUT_SECONDS = 2.0;

    private static final int CANCODER_PERIOD_MS = 50;

    private static int instances = 0;

    private final int instanceId;
    private final DoubleTelemetryEntry driveVelocitySetpointEntry;
    private final DoubleTelemetryEntry nextDriveVelocitySetpointEntry;
    private final BooleanTelemetryEntry openLoopEntry;
    private final DoubleTelemetryEntry steerPositionGoalEntry;
    private final BooleanTelemetryEntry activeSteerEntry;
    private final DoubleTelemetryEntry actualDriveVelocityEntry;
    private final DoubleTelemetryEntry actualSteerPositionEntry;
    private final DoubleTelemetryEntry actualSteerVelocityEntry;
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
    private final TelemetryTalonFX steerMotor;
    private final CANCoder absoluteSteerEncoder;

    private final double driveMotorConversionFactorPosition;
    private final double driveMotorConversionFactorVelocity;
    private final double steerMotorConversionFactorPosition;
    private final double steerMotorConversionFactorVelocity;
    private final double nominalVoltage;
    private final double steerEncoderOffsetRadians;

    private final TunablePIDGains driveVelocityPIDGains;
    private final TunableFFGains driveVelocityFFGains;
    private final TunablePIDGains steerPositionPIDGains;
    private SimpleMotorFeedforward driveMotorFF;

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
        steerPositionGoalEntry = new DoubleTelemetryEntry(tableName + "steerPositionGoal", true);
        activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", tuningMode);
        actualDriveVelocityEntry = new DoubleTelemetryEntry(tableName + "actualDriveVelocity", true);
        actualSteerPositionEntry = new DoubleTelemetryEntry(tableName + "actualSteerPosition", true);
        actualSteerVelocityEntry = new DoubleTelemetryEntry(tableName + "actualSteerVelocity", tuningMode);
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
        ;

        this.steerPositionPIDGains = config.sharedConfiguration().steerPositionPIDGains();
        ;

        // Drive motor
        this.driveMotor = new TelemetryTalonFX(
                config.driveMotorPort(),
                tableName + "driveMotor",
                config.sharedConfiguration().canBus());
        configDriveMotor(config);

        // Steer encoder
        this.absoluteSteerEncoder = new CANCoder(
                config.steerEncoderPort(), config.sharedConfiguration().canBus());
        configSteerEncoder(config);

        // Steer motor
        this.steerMotor = new TelemetryTalonFX(
                config.steerMotorPort(),
                tableName + "steerMotor",
                config.sharedConfiguration().canBus());
        configSteerMotor(config);

        this.nominalVoltage = config.sharedConfiguration().nominalVoltage();
        this.steerEncoderOffsetRadians = config.offsetRadians();

        this.driveMotorFF = driveVelocityFFGains.createFeedforward();

        resetSteerToAbsolute(INITIAL_TIMEOUT_SECONDS);
    }

    private void configDriveMotor(SwerveModuleConfiguration config) {
        double startTime = Timer.getFPGATimestamp();
        boolean faultInitializing;
        do {
            faultInitializing = checkCTREErrorSilent(driveMotor.configFactoryDefault(CAN_TIMEOUT_MS));

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            applyCommonMotorConfiguration(motorConfiguration, config);

            motorConfiguration.supplyCurrLimit.currentLimit =
                    config.sharedConfiguration().driveContinuousCurrentLimit();
            motorConfiguration.supplyCurrLimit.triggerThresholdCurrent =
                    config.sharedConfiguration().drivePeakCurrentLimit();
            motorConfiguration.supplyCurrLimit.triggerThresholdTime =
                    config.sharedConfiguration().drivePeakCurrentDurationSeconds();

            config.sharedConfiguration().driveVelocityPIDGains().setSlot(motorConfiguration.slot0);

            faultInitializing |= checkCTREErrorSilent(driveMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS));

            driveMotor.setInverted(
                    config.driveMotorInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);

            faultInitializing |= checkCTREErrorSilent(
                    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS));

            driveMotor.setSensorPhase(false);

            driveMotor.setNeutralMode(NeutralMode.Brake);

            // Clear the reset of it starting up
        } while (faultInitializing && Timer.getFPGATimestamp() - startTime <= SwerveModule.INITIAL_TIMEOUT_SECONDS);

        driveMotor.hasResetOccurred();
        driveMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void configSteerMotor(SwerveModuleConfiguration config) {
        double startTime = Timer.getFPGATimestamp();
        boolean faultInitializing;
        do {
            faultInitializing = checkCTREErrorSilent(steerMotor.configFactoryDefault());

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            applyCommonMotorConfiguration(motorConfiguration, config);

            motorConfiguration.supplyCurrLimit.currentLimit =
                    config.sharedConfiguration().steerContinuousCurrentLimit();
            motorConfiguration.supplyCurrLimit.triggerThresholdCurrent =
                    config.sharedConfiguration().steerPeakCurrentLimit();
            motorConfiguration.supplyCurrLimit.triggerThresholdTime =
                    config.sharedConfiguration().steerPeakCurrentDurationSeconds();

            config.sharedConfiguration().steerPositionPIDGains().setSlot(motorConfiguration.slot0);
            motorConfiguration.slot0.allowableClosedloopError =
                    config.sharedConfiguration().allowableSteerErrorRadians() / steerMotorConversionFactorPosition;
            // Max control effort of 7 volts
            motorConfiguration.slot0.closedLoopPeakOutput =
                    7.0 / config.sharedConfiguration().nominalVoltage();

            faultInitializing |= checkCTREErrorSilent(steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS));

            steerMotor.setInverted(
                    config.steerMotorInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);

            faultInitializing |= checkCTREErrorSilent(
                    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS));

            // Because + on motor is clockwise, and we want + on encoder to be
            // counter-clockwise we have to set the sensor phase
            steerMotor.setSensorPhase(false);

            steerMotor.setNeutralMode(NeutralMode.Brake);

        } while (faultInitializing && Timer.getFPGATimestamp() - startTime <= SwerveModule.INITIAL_TIMEOUT_SECONDS);

        // Clear the reset of it starting up
        steerMotor.hasResetOccurred();

        steerMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void applyCommonMotorConfiguration(
            TalonFXConfiguration motorConfiguration, SwerveModuleConfiguration config) {
        motorConfiguration.supplyCurrLimit.enable = true;
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration().nominalVoltage();
    }

    private void configSteerEncoder(SwerveModuleConfiguration config) {
        double startTime = Timer.getFPGATimestamp();
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.sensorDirection = config.steerEncoderInverted();
        encoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        boolean faultInitializing;
        do {
            faultInitializing =
                    checkCTREErrorSilent(absoluteSteerEncoder.configAllSettings(encoderConfiguration, CAN_TIMEOUT_MS));

            // Because we are only reading this at the beginning we do not have to update it
            // often
            faultInitializing |= checkCTREErrorSilent(absoluteSteerEncoder.setStatusFramePeriod(
                    CANCoderStatusFrame.SensorData, CANCODER_PERIOD_MS, CAN_TIMEOUT_MS));
        } while (faultInitializing && Timer.getFPGATimestamp() - startTime <= SwerveModule.INITIAL_TIMEOUT_SECONDS);

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

    /**
     * @param errorCode the error code passed by CTRE
     * @param message the message explaining the error
     * @return true of there was an error, false if there wasn't
     */
    private boolean checkCTREError(ErrorCode errorCode, String message) {
        if (checkCTREErrorSilent(errorCode)) {
            String fullMessage = String.format("%s: %s", message, errorCode.toString());

            reportError(fullMessage);
            return true;
        }
        return false;
    }

    private boolean checkCTREErrorSilent(ErrorCode errorCode) {
        return errorCode != ErrorCode.OK;
    }

    private void checkForSteerMotorReset() {
        // Steer motor lost power
        if (RobotBase.isReal() && steerMotor.hasResetOccurred()) {
            reportError("Steer motor reset occurred");
            setToAbsolute = false;
            resetSteerToAbsolute();
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
            if (!checkCTREErrorSilent(absoluteSteerEncoder.getLastError())) {
                gotAbsolutePosition = true;
            }
            if (gotAbsolutePosition) {
                ErrorCode settingPositionError = steerMotor.setSelectedSensorPosition(
                        absolutePosition / steerMotorConversionFactorPosition, 0, CAN_TIMEOUT_MS);
                // If no error
                if (!checkCTREErrorSilent(settingPositionError)) {
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
                Units.degreesToRadians(absoluteSteerEncoder.getAbsolutePosition()) + steerEncoderOffsetRadians);
    }

    private double getSteerAngleRadiansNoWrap() {
        return steerMotor.getSelectedSensorPosition() * steerMotorConversionFactorPosition;
    }

    /**
     * @return the rotation of the wheel
     */
    private Rotation2d getSteerAngle() {
        return new Rotation2d(MathUtil.angleModulus(getSteerAngleRadiansNoWrap()));
    }

    private double getSteerVelocityRadiansPerSecond() {
        return steerMotor.getSelectedSensorVelocity() * steerMotorConversionFactorVelocity;
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

        steerMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
        driveMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void resetDriveMotorPosition() {
        driveMotor.setSelectedSensorPosition(0.0);
    }

    public CommandBase getToggleDeadModeCommand() {
        return Commands.runOnce(() -> setDeadModule(!isDeadMode))
                .ignoringDisable(true)
                .withName("Toggle");
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

    /**
     * Set the desired state for this swerve module
     *
     * @param state the desired state
     * @param nextState the next desired state to be used for acceleration feedforward.
     * @param activeSteer if steer should be active
     * @param openLoop if velocity control should be feed forward only. False if to use PIDF for
     *     velocity control.
     */
    public void setDesiredState(
            SwerveModuleState state, SwerveModuleState nextState, boolean activeSteer, boolean openLoop) {
        Robot.startWNode("SwerveModule[" + instanceId + "]#setDesiredState");
        Robot.startWNode("checkForResetAndGains");
        checkForSteerMotorReset();
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
        // Assume perfect following, that we will reach our desired state by the time we
        // have to use our next one
        nextState = SwerveModuleState.optimize(nextState, state.angle);

        Robot.startWNode("setDriveState");
        setDriveReference(state.speedMetersPerSecond, nextState.speedMetersPerSecond, openLoop);
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
        steerMotor.set(TalonFXControlMode.PercentOutput, steerVolts / nominalVoltage);
    }

    private void setSteerReference(double targetAngleRadians, boolean activeSteer) {
        activeSteerEntry.append(activeSteer);
        steerPositionGoalEntry.append(Units.radiansToDegrees(targetAngleRadians));

        if (activeSteer) {
            steerMotor.set(
                    TalonFXControlMode.Position,
                    RaiderMathUtils.calculateContinuousInputSetpoint(getSteerAngleRadiansNoWrap(), targetAngleRadians)
                            / steerMotorConversionFactorPosition);
        } else {
            steerMotor.neutralOutput();
        }
    }

    private void setDriveReference(
            double targetVelocityMetersPerSecond, double nextTargetVelocityMetersPerSecond, boolean openLoop) {
        driveVelocitySetpointEntry.append(targetVelocityMetersPerSecond);
        nextDriveVelocitySetpointEntry.append(nextTargetVelocityMetersPerSecond);
        openLoopEntry.append(openLoop);

        double feedforwardValuePercent =
                driveMotorFF.calculate(targetVelocityMetersPerSecond, nextTargetVelocityMetersPerSecond, Constants.DT)
                        / nominalVoltage;

        if (openLoop) {
            driveMotor.set(TalonFXControlMode.PercentOutput, feedforwardValuePercent);
        } else {
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
            steerMotor.configureSlot(newSlotConfig);

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
                && Math.abs(steerMotor.getSelectedSensorVelocity() * steerMotorConversionFactorVelocity)
                        < Units.degreesToRadians(0.5);
    }

    /** Log all telemetry values. Should be called (only) in subsystem periodic */
    public void logValues() {
        driveMotor.logValues();
        steerMotor.logValues();
        actualSteerPositionEntry.append(getSteerAngle().getDegrees());
        actualSteerVelocityEntry.append(Units.radiansToDegrees(getSteerVelocityRadiansPerSecond()));
        actualDriveVelocityEntry.append(getDriveVelocityMetersPerSecond());
        absoluteHeadingEntry.append(Units.radiansToDegrees(getAbsoluteRadians()));
        setToAbsoluteEntry.append(setToAbsolute);
    }
}
