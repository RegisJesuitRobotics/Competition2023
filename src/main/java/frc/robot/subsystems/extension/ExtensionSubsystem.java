package frc.robot.subsystems.extension;

import static frc.robot.Constants.ExtensionConstants.*;
import static frc.robot.utils.RaiderUtils.checkRevError;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;

/**
 * <strong>Do not use this directly in commands, use {@link frc.robot.subsystems.LiftExtensionSuperStructure}</strong>
 */
public class ExtensionSubsystem extends SubsystemBase {
    enum Mode {
        CLOSED_LOOP(1),
        RAW_VOLTAGE(2);

        final int logValue;

        Mode(int logValue) {
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
    private final DoubleTelemetryEntry leftPosition =
            new DoubleTelemetryEntry("/extension/leftPosition", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry leftVelocity =
            new DoubleTelemetryEntry("/extension/leftVelocity", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry rightPosition =
            new DoubleTelemetryEntry("/extension/rightPosition", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry rightVelocity =
            new DoubleTelemetryEntry("/extension/rightVelocity", MiscConstants.TUNING_MODE);

    private double voltage = 0.0;
    private Mode currentMode = Mode.CLOSED_LOOP;

    public ExtensionSubsystem() {
        configMotors();

        eventEntry.append("Extension initialized");
    }

    private void configMotors() {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing = false;

        do {
            faultInitializing |= checkRevError(leftMotor.restoreFactoryDefaults());
            faultInitializing |= checkRevError(rightMotor.restoreFactoryDefaults());

            leftMotor.setInverted(INVERT_LEADER);
            faultInitializing |= checkRevError(rightMotor.follow(leftMotor, INVERT_FOLLOWER_FROM_LEADER));

            double conversionFactor = (Math.PI * ROLLER_DIAMETER_METERS) / GEAR_REDUCTION;
            faultInitializing |= checkRevError(leftEncoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(leftEncoder.setVelocityConversionFactor(conversionFactor));

            faultInitializing |= checkRevError(leftMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
            faultInitializing |=
                    checkRevError(rightMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        if (faultInitializing) {
            eventEntry.append("Motors timed out initializing");
        }
        failedConfigurationAlert.set(faultInitializing);
    }

    public void setDistance(double distanceMeters) {
        currentMode = Mode.CLOSED_LOOP;
        controller.setGoal(distanceMeters);
    }

    public void setVoltage(double voltage) {
        currentMode = Mode.RAW_VOLTAGE;
        this.voltage = voltage;
    }

    public double getDistance() {
        return leftEncoder.getPosition();
    }

    public void stopMovement() {
        setVoltage(0.0);
    }

    @Override
    public void periodic() {
        Robot.startWNode("ExtensionSubsystem#periodic");

        if (currentMode == Mode.RAW_VOLTAGE) {
            leftMotor.setVoltage(voltage);
        } else if (currentMode == Mode.CLOSED_LOOP) {
            double feedbackOutput = controller.calculate(getDistance());

            TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
            leftMotor.setVoltage(feedbackOutput + feedforward.calculate(currentSetpoint.velocity));
        }

        Robot.startWNode("LogValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        leftMotor.logValues();
        rightMotor.logValues();
        leftPosition.append(leftEncoder.getPosition());
        rightPosition.append(rightEncoder.getPosition());
        leftVelocity.append(leftEncoder.getVelocity());
        rightVelocity.append(rightEncoder.getVelocity());
        modeEntry.append(currentMode.logValue);

        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
    }
}
