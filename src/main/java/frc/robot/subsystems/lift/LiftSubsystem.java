package frc.robot.subsystems.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    private final CANSparkMax topLeftMotor =
            new CANSparkMax(LiftConstants.TOP_LEFT_MOTOR_PORT, MotorType.kBrushless);

    private final CANSparkMax bottomLeftMotor =
            new CANSparkMax(LiftConstants.BOTTOM_LEFT_MOTOR_PORT, MotorType.kBrushless);

    private final CANSparkMax topRightMotor =
            new CANSparkMax(LiftConstants.TOP_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final CANSparkMax bottomRightMotor =
            new CANSparkMax(LiftConstants.BOTOM_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    public LiftSubsystem() {
    }
}
