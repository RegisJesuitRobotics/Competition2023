package frc.robot.subsystems.lift;

import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.utils.RaiderUtils.checkRevError;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;

public class LiftSubsystem extends SubsystemBase {
    private final TelemetryCANSparkMax leftMotor =
            new TelemetryCANSparkMax(LEFT_MOTOR_PORT, MotorType.kBrushless, "/lifter/leftMotor");

    private final TelemetryCANSparkMax rightMotor =
            new TelemetryCANSparkMax(RIGHT_MOTOR_PORT, MotorType.kBrushless, "/lifter/rightMotor");

    private final RelativeEncoder encoder = leftMotor.getEncoder();

    private final TunableTelemetryProfiledPIDController trapezoidalPidController =
            new TunableTelemetryProfiledPIDController("/lifter/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    private final LiftMechanism2d mechanism2d = new LiftMechanism2d();
    private ArmFeedforward feedforward = FF_GAINS.createFeedforward();

    private final Alert failedConfigurationAlert = new Alert("Lifter Arm Failed to Configure Motor", AlertType.ERROR);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/lifter/events");

    public LiftSubsystem() {
        SendableTelemetryManager.getInstance().addSendable("Lifter", mechanism2d.getMechanism2dObject());

        configMotors();
        eventEntry.append("Lifter initialized");
    }

    private void configMotors() {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing = false;

        do {
            faultInitializing |= checkRevError(leftMotor.restoreFactoryDefaults());
            faultInitializing |= checkRevError(rightMotor.restoreFactoryDefaults());

            leftMotor.setInverted(INVERT_LEADER);
            faultInitializing |= checkRevError(rightMotor.follow(leftMotor, INVERT_FOLLOWER_FROM_LEADER));

            double conversionFactor = (Math.PI * 2) / GEAR_REDUCTION;
            faultInitializing |= checkRevError(encoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(encoder.setVelocityConversionFactor(conversionFactor));

            faultInitializing |= checkRevError(leftMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
            faultInitializing |=
                    checkRevError(rightMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
        } while (configTimeout.hasNotTimedOut() && faultInitializing);

        if (faultInitializing) {
            eventEntry.append("Motors timed out initializing");
        }
        failedConfigurationAlert.set(faultInitializing);

        leftMotor.burnFlashIfShould();
        rightMotor.burnFlashIfShould();
    }

    /**
     * @param angle the rotation relative to the default frame perimeter position
     */
    public void setArmAngle(Rotation2d angle) {
        // Limited to physical constraints
        trapezoidalPidController.setGoal(MathUtil.clamp(
                MathUtil.angleModulus(angle.getRadians()), MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians()));
    }

    /**
     * @return the rotation from the default frame perimeter position
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    /**
     * Set the height of the claw relative to the floor
     * @param heightMeters the height of the claw relative to the floor
     */
    public void setClawHeight(double heightMeters) {
        double relativeHeight = VERTICAL_BAR_HEIGHT_FROM_FLOOR - (heightMeters + HORIZONTAL_BAR_TO_CLAW);
        double angleRadians = Math.atan(relativeHeight / HORIZONTAL_BAR_LENGTH);
        setArmAngle(Rotation2d.fromRadians(angleRadians));
    }

    @Override
    public void periodic() {
        Robot.startWNode("LifterSubsystem#periodic");

        double feedbackOutput = trapezoidalPidController.calculate(getArmAngle().getRadians());

        TrapezoidProfile.State currentSetpoint = trapezoidalPidController.getSetpoint();
        leftMotor.setVoltage(
                feedbackOutput + feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity));

        logValues();
        Robot.endWNode();
    }

    private void logValues() {
        Robot.startWNode("logValues");
        mechanism2d.update(getArmAngle());
        leftMotor.logValues();
        rightMotor.logValues();

        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
        Robot.endWNode();
    }
}
