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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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
import frc.robot.utils.DualHomeable;

public class LiftSubsystem extends SubsystemBase implements DualHomeable {
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
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ABSOLUTE_ENCODER_PORT);
    private final Encoder relativeEncoder =
            new Encoder(RELATIVE_ENCODER_PORT_A, RELATIVE_ENCODER_PORT_B, INVERT_RELATIVE_ENCODER);

    private final TunableTelemetryProfiledPIDController controller =
            new TunableTelemetryProfiledPIDController("/lifter/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    private ArmFeedforward feedforward = FF_GAINS.createFeedforward();

    private final LiftMechanism2d mechanism2d = new LiftMechanism2d(new Color8Bit(255, 0, 0));
    private final LiftMechanism2d setpointMechanism2d = new LiftMechanism2d(new Color8Bit(0, 255, 0));

    private final Alert failedConfigurationAlert = new Alert("Lifter Arm Failed to Configure Motor", AlertType.ERROR);
    private final Alert absoluteEncoderNotConnectedAlert =
            new Alert("Lifter Absolute Encoder is Not Connected. Cannot Zero", AlertType.ERROR);
    private final BooleanTelemetryEntry isZeroedEntry = new BooleanTelemetryEntry("/lifter/isZeroed", false);
    private final BooleanTelemetryEntry absoluteEncoderConnectedEntry =
            new BooleanTelemetryEntry("/lifter/absoluteEncoderConnected", false);
    private final BooleanTelemetryEntry relativeEncoderConnectedEntry =
            new BooleanTelemetryEntry("/lifter/relativeEncoderStopped", false);
    private final DoubleTelemetryEntry absoluteEncoderEntry =
            new DoubleTelemetryEntry("/lifter/absoluteEncoder", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry relativeEncoderEntry =
            new DoubleTelemetryEntry("/lifter/relativeEncoderPosition", MiscConstants.TUNING_MODE);
    private final DoubleTelemetryEntry relativeEncoderVelocityEntry =
            new DoubleTelemetryEntry("/lifter/relativeEncoderVelocity", MiscConstants.TUNING_MODE);
    private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/lifter/events");
    private final IntegerTelemetryEntry modeEntry = new IntegerTelemetryEntry("/lifter/mode", false);
    private final DoubleTelemetryEntry leftRawVoltageRequestEntry =
            new DoubleTelemetryEntry("/lifter/leftVoltageRequest", false);
    private final DoubleTelemetryEntry rightRawVoltageRequestEntry =
            new DoubleTelemetryEntry("/lifter/leftVoltageRequest", false);

    private boolean isZeroed = false;
    private double relativeEncoderOffsetRadians = 0.0;

    private LiftControlMode currentMode = LiftControlMode.RAW_VOLTAGE;
    // Only for voltage mode
    private double desiredLeftVoltage = 0.0;
    private double desiredRightVoltage = 0.0;

    public LiftSubsystem() {
        SendableTelemetryManager.getInstance()
                .addSendable("/lifter/LifterMechanism2d", mechanism2d.getMechanism2dObject());
        SendableTelemetryManager.getInstance()
                .addSendable("/lifter/LifterSetpointMechanism2d", setpointMechanism2d.getMechanism2dObject());

        absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        relativeEncoder.setDistancePerPulse((Math.PI * 2) / 2048.0);

        configMotors();
        controller.setTolerance(POSITION_TOLERANCE_RADIANS, VELOCITY_TOLERANCE_RADIANS_SECOND);
        eventEntry.append("Lifter initialized");
    }

    private void configMotors() {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing = false;

        do {
            faultInitializing |= checkRevError(leftMotor.restoreFactoryDefaults());
            faultInitializing |= checkRevError(rightMotor.restoreFactoryDefaults());

            leftMotor.setInverted(INVERT_LEFT);
            rightMotor.setInverted(INVERT_RIGHT);

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

    public double getEstimatedTimeForPosition(Rotation2d position) {
        return new TrapezoidProfile(
                        TRAPEZOIDAL_PROFILE_GAINS.createConstraints(),
                        new TrapezoidProfile.State(position.getRadians(), 0.0),
                        new TrapezoidProfile.State(getArmAngle().getRadians(), getVelocity()))
                .totalTime();
    }

    /**
     * @param angle the rotation in the robot's frame of reference (0 is parallel to the floor)
     */
    public void setDesiredArmAngle(Rotation2d angle) {
        if (currentMode != LiftControlMode.CLOSED_LOOP) {
            controller.reset(getArmAngle().getRadians(), getVelocity());
        }
        currentMode = LiftControlMode.CLOSED_LOOP;
        // Limited to physical constraints
        controller.setGoal(MathUtil.clamp(angle.getRadians(), MIN_ANGLE.getRadians(), MAX_ANGLE.getRadians()));
    }

    public boolean atClosedLoopGoal() {
        return currentMode != LiftControlMode.CLOSED_LOOP || controller.atGoal();
    }

    public void setVoltage(double voltage) {
        setVoltage(voltage, voltage);
    }

    @Override
    public void setInHome() {
        setEncoderPosition(MIN_ANGLE);
        isZeroed = true;
        eventEntry.append("Homed mechanism");
    }

    private void setEncoderPosition(Rotation2d position) {
        leftEncoder.setPosition(position.getRadians());
        rightEncoder.setPosition(position.getRadians());
        controller.reset(position.getRadians(), leftEncoder.getVelocity());
    }

    @Override
    public double getLeftCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public double getRightCurrent() {
        return rightMotor.getOutputCurrent();
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        currentMode = LiftControlMode.RAW_VOLTAGE;
        this.desiredLeftVoltage = leftVoltage;
        this.desiredRightVoltage = rightVoltage;
    }

    public void stopMovement() {
        // Only if we are homed do we hold the position with FF
        //        setDesiredArmAngle(getArmAngle());
        setVoltage(0.0);
    }

    /**
     * @return the rotation from the default frame perimeter position
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRadians(getRelativeEncoder());
    }

    private double getAbsoluteEncoder() {
        return Units.rotationsToRadians(absoluteEncoder.get()) * (INVERT_ABSOLUTE_ENCODER ? -1 : 1)
                + ABSOLUTE_ENCODER_OFFSET_FROM_ZERO;
    }

    private double getRelativeEncoder() {
        return relativeEncoder.getDistance() + relativeEncoderOffsetRadians;
    }

    public double getVelocity() {
        return relativeEncoder.getRate();
    }

    @Override
    public void periodic() {
        Robot.startWNode("LifterSubsystem#periodic");

        if (DriverStation.isDisabled()) {
            setVoltage(0.0);
        }

        if (!isZeroed) {
            if (absoluteEncoder.isConnected()) {
                double absoluteEncoderRadians = getAbsoluteEncoder();
                relativeEncoderOffsetRadians = absoluteEncoderRadians - getRelativeEncoder();

                leftEncoder.setPosition(absoluteEncoderRadians);
                rightEncoder.setPosition(absoluteEncoderRadians);

                isZeroed = true;
            }
        }

        if (currentMode == LiftControlMode.RAW_VOLTAGE) {
            leftMotor.setVoltage(desiredLeftVoltage);
            rightMotor.setVoltage(desiredRightVoltage);
        } else if (currentMode == LiftControlMode.CLOSED_LOOP) {
            double feedbackOutput = controller.calculate(getArmAngle().getRadians());

            setpointMechanism2d.setAngle(Rotation2d.fromRadians(controller.getSetpoint().position));

            TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
            double combinedOutput =
                    feedbackOutput + feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity);
            leftMotor.setVoltage(combinedOutput);
            rightMotor.setVoltage(combinedOutput);
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
        absoluteEncoderEntry.append(getAbsoluteEncoder());
        relativeEncoderEntry.append(getRelativeEncoder());
        relativeEncoderVelocityEntry.append(getVelocity());

        modeEntry.append(currentMode.logValue);

        leftRawVoltageRequestEntry.append(desiredLeftVoltage);
        rightRawVoltageRequestEntry.append(desiredRightVoltage);

        absoluteEncoderNotConnectedAlert.set(!absoluteEncoder.isConnected());
        absoluteEncoderConnectedEntry.append(absoluteEncoder.isConnected());
        relativeEncoderConnectedEntry.append(relativeEncoder.getStopped());

        isZeroedEntry.append(isZeroed);

        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
    }
}
