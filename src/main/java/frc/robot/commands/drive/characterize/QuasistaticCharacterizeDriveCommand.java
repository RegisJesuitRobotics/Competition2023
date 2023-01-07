package frc.robot.commands.drive.characterize;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class QuasistaticCharacterizeDriveCommand extends CharacterizeDriveCommand {
    private final double rampRateVoltsSecond;

    public QuasistaticCharacterizeDriveCommand(double rampRateVoltsSecond, SwerveDriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        this.rampRateVoltsSecond = rampRateVoltsSecond;
    }

    @Override
    protected double getVoltage(double currentTime) {
        return currentTime * rampRateVoltsSecond;
    }
}
