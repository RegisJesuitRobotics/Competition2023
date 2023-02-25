package frc.robot.subsystems.extension;

import static frc.robot.Constants.ExtensionConstants.*;
import static frc.robot.utils.RaiderUtils.checkRevError;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;
import frc.robot.utils.DualHomeable;

public class ExtensionSubsystem extends SubsystemBase implements DualHomeable {
    enum ExtensionControlMode {
        CLOSED_LOOP(1),
        RAW_VOLTAGE(2);

        final int logValue;

        ExtensionControlMode(int logValue) {
            this.logValue = logValue;
        }
    }

    private final TelemetryCANSparkMax leftMotor = new TelemetryCANSparkMax(
            LEFT_MOTOR_PORT, MotorType.kBrushless, "/extension/left", MiscConstants.TUNING_MODE);
    private final TelemetryCANSparkMax rightMotor = new TelemetryCANSparkMax(
            RIGHT_MOTOR_PORT, MotorType.kBrushless, "/extension/right", MiscConstants.TUNING_MODE);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final TunableTelemetryProfiledPIDController controller =
            new TunableTelemetryProfiledPIDController("/extension/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    private SimpleMotorFeedforward feedforward = FF_GAINS.createFeedforward();

    private final Alert failedConfigurationAlert = new Alert("Extension Failed to Configure Motor", AlertType.ERROR);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/extension/events");
    private final IntegerTelemetryEntry modeEntry = new IntegerTelemetryEntry("/extension/mode", false);
    private final BooleanTelemetryEntry homedEntry = new BooleanTelemetryEntry("/extension/homed", false);
    private final DoubleTelemetryEntry leftRawVoltageRequestEntry =
            new DoubleTelemetryEntry("/extension/leftVoltageRequest", false);
    private final DoubleTelemetryEntry rightRawVoltageRequestEntry =
            new DoubleTelemetryEntry("/extension/rightVoltageRequest", false);
    private final Alert notHomedAlert = new Alert("Extension is Not Homed!", AlertType.WARNING);

    private boolean isHomed = false;
    private double leftVoltage = 0.0;
    private double rightVoltage = 0.0;
    private ExtensionControlMode currentMode = ExtensionControlMode.RAW_VOLTAGE;

    public ExtensionSubsystem() {
        configMotors();

        controller.setTolerance(POSITION_TOLERANCE_METERS, VELOCITY_TOLERANCE_METERS_SECOND);
        eventEntry.append("Extension initialized");
    }

    private void configMotors() {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing = false;

        do {
            faultInitializing |= checkRevError(leftMotor.restoreFactoryDefaults());
            faultInitializing |= checkRevError(rightMotor.restoreFactoryDefaults());

            leftMotor.setInverted(INVERT_LEFT);
            rightMotor.setInverted(INVERT_RIGHT);

            double conversionFactor = (METERS_PER_REV) / GEAR_REDUCTION;
            faultInitializing |= checkRevError(leftEncoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(leftEncoder.setVelocityConversionFactor(conversionFactor / 60));
            faultInitializing |= checkRevError(rightEncoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(rightEncoder.setVelocityConversionFactor(conversionFactor / 60));

            faultInitializing |= checkRevError(leftMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
            faultInitializing |=
                    checkRevError(rightMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        if (faultInitializing) {
            eventEntry.append("Motors timed out initializing");
        }
        failedConfigurationAlert.set(faultInitializing);
    }

    public double getEstimatedTimeForPosition(double position) {
        return new TrapezoidProfile(
                        TRAPEZOIDAL_PROFILE_GAINS.createConstraints(),
                        new TrapezoidProfile.State(position, 0.0),
                        new TrapezoidProfile.State(getPosition(), getVelocity()))
                .totalTime();
    }

    public void setDesiredPosition(double distanceMeters) {
        if (!isHomed) {
            setVoltage(0.0);
            return;
        }
        if (currentMode != ExtensionControlMode.CLOSED_LOOP) {
            controller.reset(getPosition(), getVelocity());
        }

        distanceMeters = MathUtil.clamp(distanceMeters, MIN_POSITION, MAX_POSITION);

        currentMode = ExtensionControlMode.CLOSED_LOOP;
        controller.setGoal(distanceMeters);
    }

    public boolean atClosedLoopGoal() {
        return currentMode != ExtensionControlMode.CLOSED_LOOP || controller.atGoal();
    }

    public void setVoltage(double voltage) {
        setVoltage(voltage, voltage);
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        currentMode = ExtensionControlMode.RAW_VOLTAGE;
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    public double getPosition() {
        return leftEncoder.getPosition();
    }

    public double getVelocity() {
        return leftEncoder.getVelocity();
    }

    private void setPosition(double position) {
        leftEncoder.setPosition(position);
        rightEncoder.setPosition(position);
    }

    @Override
    public void setInHome() {
        setPosition(0.0);
        isHomed = true;
    }

    @Override
    public double getLeftCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public double getRightCurrent() {
        return rightMotor.getOutputCurrent();
    }

    public void stopMovement() {
        setVoltage(0.0);
    }

    @Override
    public void periodic() {
        Robot.startWNode("ExtensionSubsystem#periodic");

        if (currentMode == ExtensionControlMode.RAW_VOLTAGE) {
            leftMotor.setVoltage(leftVoltage);
            rightMotor.setVoltage(rightVoltage);
        } else if (currentMode == ExtensionControlMode.CLOSED_LOOP) {
            double feedbackOutput = controller.calculate(getPosition());

            TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
            double combinedOutput = feedbackOutput + feedforward.calculate(currentSetpoint.velocity);
            leftMotor.setVoltage(combinedOutput);
            rightMotor.setVoltage(combinedOutput);
        }

        Robot.startWNode("LogValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        leftMotor.logValues();
        rightMotor.logValues();
        modeEntry.append(currentMode.logValue);
        homedEntry.append(isHomed);
        notHomedAlert.set(!isHomed);
        leftRawVoltageRequestEntry.append(leftVoltage);
        rightRawVoltageRequestEntry.append(rightVoltage);

        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
    }
}
