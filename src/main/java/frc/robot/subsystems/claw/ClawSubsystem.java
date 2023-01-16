package frc.robot.subsystems.claw;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final DoubleSolenoid leftSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_SOLENOID_PORTS[0], LEFT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid rightSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_SOLENOID_PORTS[0], RIGHT_SOLENOID_PORTS[1]);

    public ClawSubsystem() {}
}
