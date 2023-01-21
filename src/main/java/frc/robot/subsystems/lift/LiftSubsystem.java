package frc.robot.subsystems.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    // TODO: update the number of motors

    private final CANSparkMax topLeftMotor =
            new CANSparkMax(LiftConstants.TOP_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax bottomLeftMotor =
            new CANSparkMax(LiftConstants.BOTTOM_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax topRightMotor =
            new CANSparkMax(LiftConstants.TOP_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax bottomRightMotor =
            new CANSparkMax(LiftConstants.BOTTOM_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    private LiftSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
}
