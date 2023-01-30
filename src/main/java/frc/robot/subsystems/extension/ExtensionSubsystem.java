package frc.robot.subsystems.extension;

import static frc.robot.Constants.ExtensionConstants.*;
import static frc.robot.utils.RaiderUtils.checkRevError;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;

public class ExtensionSubsystem extends SubsystemBase {
    private final TelemetryCANSparkMax leftMotor =
            new TelemetryCANSparkMax(LEFT_MOTOR_PORT, MotorType.kBrushless, "/extension/left");
    private final TelemetryCANSparkMax rightMotor =
            new TelemetryCANSparkMax(RIGHT_MOTOR_PORT, MotorType.kBrushless, "/extension/right");
    private final RelativeEncoder encoder = leftMotor.getEncoder();

    private final Alert failedConfigurationAlert = new Alert("Extension Failed to Configure Motor", AlertType.ERROR);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/lifter/events");

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

            double conversionFactor = (Math.PI * ROLLER_DIAMETER) / GEAR_REDUCTION;
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

    @Override
    public void periodic() {
        logValues();
    }

    private void logValues() {
        leftMotor.logValues();
        rightMotor.logValues();
    }
}
