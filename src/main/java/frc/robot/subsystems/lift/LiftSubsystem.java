package frc.robot.subsystems.lift;

import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.utils.RaiderUtils.checkRevError;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
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

    private final LiftMechanism2d mechanism2d = new LiftMechanism2d(new Color8Bit(255, 0, 0));
    private final LiftMechanism2d setpointMechanism2d = new LiftMechanism2d(new Color8Bit(0, 255, 0));

    private final Alert failedConfigurationAlert = new Alert("Lifter Arm Failed to Configure Motor", AlertType.ERROR);
    private final Alert notHomedAlert = new Alert("Lifter is Not Homed!", AlertType.WARNING);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/lifter/events");
    private final IntegerTelemetryEntry modeEntry = new IntegerTelemetryEntry("/lifter/mode", false);
    private final BooleanTelemetryEntry homedEntry = new BooleanTelemetryEntry("/lifter/homed", true);
    private final DoubleTelemetryEntry rawVoltageVoltageEntry =
            new DoubleTelemetryEntry("/lifter/rawVoltageVoltage", false);

    private LiftControlMode currentMode = LiftControlMode.RAW_VOLTAGE;
    // Only for voltage mode
    private double desiredVoltage = 0.0;
    private boolean isHomed = false;

    public LiftSubsystem() {
        SendableTelemetryManager.getInstance()
                .addSendable("/lifter/LifterMechanism2d", mechanism2d.getMechanism2dObject());
        SendableTelemetryManager.getInstance()
                .addSendable("/lifter/LifterSetpointMechanism2d", setpointMechanism2d.getMechanism2dObject());

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
            faultInitializing |= checkRevError(leftEncoder.setVelocityConversionFactor(conversionFactor / 60));

            faultInitializing |= checkRevError(rightEncoder.setPositionConversionFactor(conversionFactor));
            faultInitializing |= checkRevError(rightEncoder.setVelocityConversionFactor(conversionFactor / 60));

            faultInitializing |= checkRevError(leftMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));
            faultInitializing |=
                    checkRevError(rightMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT));

            faultInitializing |= checkRevError(leftMotor.setIdleMode(IdleMode.kBrake));
            faultInitializing |= checkRevError(rightMotor.setIdleMode(IdleMode.kBrake));
        } while (configTimeout.hasNotTimedOut() && faultInitializing);

        if (faultInitializing) {
            eventEntry.append("Motors timed out initializing");
        }
        failedConfigurationAlert.set(faultInitializing);

        leftMotor.burnFlashIfShould();
        rightMotor.burnFlashIfShould();
    }

    /**
     * @param angle the rotation in the robot's frame of reference (0 is parallel to the floor)
     */
    public void setDesiredArmAngle(Rotation2d angle) {
        if (currentMode != LiftControlMode.CLOSED_LOOP) {
            controller.reset(getArmAngle().getRadians(), leftEncoder.getVelocity());
        }
        currentMode = LiftControlMode.CLOSED_LOOP;
        // Limited to physical constraints
        controller.setGoal(MathUtil.clamp(angle.getRadians(), MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians()));
    }

    public boolean atClosedLoopGoal() {
        return currentMode != LiftControlMode.CLOSED_LOOP || controller.atGoal();
    }

    public void setVoltage(double voltage) {
        currentMode = LiftControlMode.RAW_VOLTAGE;
        this.desiredVoltage = voltage;
    }

    @Override
    public void setInHome() {
        setEncoderPosition(MIN_ANGLE);
        isHomed = true;
        eventEntry.append("Homed mechanism");
    }

    public void setEncoderPosition(Rotation2d position) {
        leftEncoder.setPosition(position.getRadians());
        rightEncoder.setPosition(position.getRadians());
        controller.reset(position.getRadians(), leftEncoder.getVelocity());
    }

    public void stopMovement() {
        // Only if we are homed do we hold the position with FF
        if (isHomed) {
            setDesiredArmAngle(getArmAngle());
        } else {
            setVoltage(0.0);
        }
    }

    /**
     * @return the rotation from the default frame perimeter position
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRadians(leftEncoder.getPosition());
    }

    @Override
    public double getCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        Robot.startWNode("LifterSubsystem#periodic");

        if (DriverStation.isDisabled()) {
            desiredVoltage = 0.0;
            currentMode = LiftControlMode.RAW_VOLTAGE;
        }

        if (currentMode == LiftControlMode.RAW_VOLTAGE) {
            leftMotor.setVoltage(desiredVoltage);
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
        rawVoltageVoltageEntry.append(desiredVoltage);

        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
    }
}
