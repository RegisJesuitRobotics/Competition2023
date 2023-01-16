package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {

    private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ClawConstants.LEFT_SOLENOID_FORWARD_PORT_CLAW,
            ClawConstants.LEFT_SOLENOID_REVERSE_PORT_CLAW);

    private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ClawConstants.RIGHT_SOLENOID_FORWARD_PORT_CLAW,
            ClawConstants.RIGHT_SOLENOID_REVERSE_PORT_CLAW);

    private ClawSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
}
