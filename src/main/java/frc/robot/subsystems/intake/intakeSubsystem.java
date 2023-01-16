package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants.IntakeConstants;

public class intakeSubsystem extends SubsystemBase {

    private final DoubleSolenoid topLeftSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            IntakeConstants.TOP_LEFT_SOLENOID_PORT_FORWARD_INTAKE,
            IntakeConstants.TOP_LEFT_SOLENOID_PORT_REVERSE_INTAKE);

    private final DoubleSolenoid bottomLeftSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            IntakeConstants.BOTTOM_LEFT_SOLENOID_PORT_FORWARD_INTAKE,
            IntakeConstants.BOTTOM_LEFT_SOLENOID_PORT_REVERSE_INTAKE);

    private final DoubleSolenoid topRightSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            IntakeConstants.TOP_RIGHT_SOLENOID_PORT_FORWARD_INTAKE,
            IntakeConstants.TOP_RIGHT_SOLENOID_PORT_REVERSE_INTAKE);

    private final DoubleSolenoid bottomRightSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            IntakeConstants.BOTTOM_RIGHT_SOLENOID_PORT_FOWARD_INTAKE,
            IntakeConstants.BOTTOM_RIGHT_SOLENOID_PORT_REVERSE_INTAKE);

    private intakeSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
}
