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
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;
import frc.robot.utils.Homeable;

public class LiftSubsystem extends SubsystemBase implements Homeable {
    enum LiftControlMode {
        CLOSED_LOOP(1),
        RAW_VOLTAGE(2);

        final int logValue;

        LiftControlMode(int logValue) {
            this.logValue = logValue;
        }
    }

    private final TelemetryCANSparkMax leftMotor = new TelemetryCANSparkMax(
            LEFT_MOTOR_PORT, MotorType.kBrushless, "/lifter/leftMotor", MiscConstants.TUNING_MODE);

    private final TelemetryCANSparkMax rightMotor = new TelemetryCANSparkMax(
            RIGHT_MOTOR_PORT, MotorType.kBrushless, "/lifter/rightMotor", MiscConstants.TUNING_MODE);

    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private final TunableTelemetryProfiledPIDController controller =
            new TunableTelemetryProfiledPIDController("/lifter/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    private ArmFeedforward feedforward = FF_GAINS.createFeedforward();

    private final LiftMechanism2d mechanism2d = new LiftMechanism2d();
    private final LiftMechanism2d setpointMechanism2d = new LiftMechanism2d();

    private final Alert failedConfigurationAlert = new Alert("Lifter Arm Failed to Configure Motor", AlertType.ERROR);
    private final Alert notHomedAlert = new Alert("Lifter is Not Homed!", AlertType.WARNING);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/lifter/events");
    private final IntegerTelemetryEntry modeEntry = new IntegerTelemetryEntry("/lifter/mode", false);
    private final BooleanTelemetryEntry homedEntry = new BooleanTelemetryEntry("/lifter/homed", true);

    private LiftControlMode currentMode = LiftControlMode.CLOSED_LOOP;
    private double voltage = 0.0;
    private boolean isHomed = false;

    public LiftSubsystem() {
        SendableTelemetryManager.getInstance().addSendable("Lifter", mechanism2d.getMechanism2dObject());
        SendableTelemetryManager.getInstance()
                .addSendable("LifterSetpoint", setpointMechanism2d.getMechanism2dObject());

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
            faultInitializing |= checkRevError(leftEncoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(leftEncoder.setVelocityConversionFactor(conversionFactor));

            faultInitializing |= checkRevError(rightEncoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(rightEncoder.setVelocityConversionFactor(conversionFactor));

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
        currentMode = LiftControlMode.CLOSED_LOOP;
        // Limited to physical constraints
        controller.setGoal(MathUtil.clamp(
                MathUtil.angleModulus(angle.getRadians()), MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians()));
    }

    public void setVoltage(double voltage) {
        currentMode = LiftControlMode.RAW_VOLTAGE;
        this.voltage = voltage;
    }

    @Override
    public void setInHome() {
        setPosition(MIN_ANGLE);
        isHomed = true;
    }

    public void setPosition(Rotation2d position) {
        leftEncoder.setPosition(position.getRadians());
        rightEncoder.setPosition(position.getRadians());
    }

    public void stopMovement() {
        setArmAngle(getArmAngle());
    }

    /**
     * @return the rotation from the default frame perimeter position
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromDegrees(leftEncoder.getPosition());
    }

    @Override
    public double getCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        Robot.startWNode("LifterSubsystem#periodic");

        if (currentMode == LiftControlMode.RAW_VOLTAGE) {
            leftMotor.setVoltage(voltage);
        } else if (currentMode == LiftControlMode.CLOSED_LOOP) {
            double feedbackOutput = controller.calculate(getArmAngle().getRadians());

            setpointMechanism2d.setAngle(Rotation2d.fromRadians(controller.getSetpoint().position));

            TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
            leftMotor.setVoltage(
                    feedbackOutput + feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity));
        }

        Robot.startWNode("logValues");
        logValues();
        Robot.endWNode();
        Robot.endWNode();
    }

    private void logValues() {
        mechanism2d.setAngle(getArmAngle());
        leftMotor.logValues();
        rightMotor.logValues();
        modeEntry.append(currentMode.logValue);
        homedEntry.append(isHomed);
        notHomedAlert.set(!isHomed);

        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
    }
}
