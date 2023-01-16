package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final DoubleSolenoid topLeftSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, TOP_LEFT_SOLENOID_PORTS[0], TOP_LEFT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid bottomLeftSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, BOTTOM_LEFT_SOLENOID_PORTS[0], BOTTOM_LEFT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid topRightSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, TOP_RIGHT_SOLENOID_PORTS[0], TOP_RIGHT_SOLENOID_PORTS[1]);

    private final DoubleSolenoid bottomRightSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, BOTTOM_RIGHT_SOLENOID_PORTS[0], BOTTOM_RIGHT_SOLENOID_PORTS[1]);

    public IntakeSubsystem() {}
}
