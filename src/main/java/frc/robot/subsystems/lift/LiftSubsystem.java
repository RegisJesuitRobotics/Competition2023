package frc.robot.subsystems.lift;

import static frc.robot.Constants.LiftConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    private final CANSparkMax topLeftMotor = new CANSparkMax(TOP_LEFT_MOTOR_PORT, MotorType.kBrushless);

    private final CANSparkMax bottomLeftMotor = new CANSparkMax(BOTTOM_LEFT_MOTOR_PORT, MotorType.kBrushless);

    private final CANSparkMax topRightMotor = new CANSparkMax(TOP_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final CANSparkMax bottomRightMotor = new CANSparkMax(BOTTOM_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final LiftMechanism2d mechanism2d = new LiftMechanism2d();

    public LiftSubsystem() {
        // TODO: Send mechanism via new telemetry stuff when that gets merged
        System.out.println(mechanism2d.getMechanism2dObject());
        SmartDashboard.putData("LifterArm", mechanism2d.getMechanism2dObject());
    }

    /**
     * @param angle the rotation relative to the default frame perimeter position
     */
    public void setArmAngle(Rotation2d angle) {
        // TODO: probably want to sanitize this angle with MathUtil.angleModulus and clamp it to physical constraints
    }

    /**
     * @return the rotation from the default frame perimeter position
     */
    public Rotation2d getArmAngle() {
        // TODO
        return Rotation2d.fromDegrees(0.0);
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
        mechanism2d.update(getArmAngle());
    }
}
