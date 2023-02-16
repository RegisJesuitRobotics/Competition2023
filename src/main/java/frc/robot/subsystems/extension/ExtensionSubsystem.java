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
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;

/**
 * <strong>Do not use this directly in commands, use {@link frc.robot.subsystems.LiftExtensionSuperStructure}</strong>
 */
public class ExtensionSubsystem extends SubsystemBase {
    private final TelemetryCANSparkMax leftMotor = new TelemetryCANSparkMax(
            LEFT_MOTOR_PORT, MotorType.kBrushless, "/extension/left", MiscConstants.TUNING_MODE);
    private final TelemetryCANSparkMax rightMotor = new TelemetryCANSparkMax(
            RIGHT_MOTOR_PORT, MotorType.kBrushless, "/extension/right", MiscConstants.TUNING_MODE);
    private final RelativeEncoder encoder = leftMotor.getEncoder();
    private final TunableTelemetryProfiledPIDController controller =
            new TunableTelemetryProfiledPIDController("/extension/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    private SimpleMotorFeedforward feedforward = FF_GAINS.createFeedforward();

    private final Alert failedConfigurationAlert = new Alert("Extension Failed to Configure Motor", AlertType.ERROR);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/extension/events");

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
            faultInitializing |= checkRevError(encoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(encoder.setVelocityConversionFactor(conversionFactor));

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
        controller.setGoal(distanceMeters);
    }

    public double getDistance() {
        return encoder.getPosition();
    }

    public void stopMovement() {
        setDistance(getDistance());
    }

    @Override
    public void periodic() {
        logValues();

        Robot.startWNode("ExtensionSubsystem#periodic");

        double feedbackOutput = controller.calculate(getDistance());

        TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
        leftMotor.setVoltage(feedbackOutput + feedforward.calculate(currentSetpoint.velocity));

        Robot.startWNode("LogValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }

        leftMotor.logValues();
        rightMotor.logValues();
    }
}
