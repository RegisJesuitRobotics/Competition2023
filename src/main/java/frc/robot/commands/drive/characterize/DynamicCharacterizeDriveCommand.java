package frc.robot.commands.drive.characterize;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DynamicCharacterizeDriveCommand extends CharacterizeDriveCommand {
    private final double voltage;

    public DynamicCharacterizeDriveCommand(double voltage, SwerveDriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        this.voltage = voltage;
    }

    @Override
    protected double getVoltage(double currentTime) {
        return voltage;
    }
}
