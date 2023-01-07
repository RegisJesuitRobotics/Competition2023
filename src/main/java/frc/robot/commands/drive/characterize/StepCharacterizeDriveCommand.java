package frc.robot.commands.drive.characterize;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class StepCharacterizeDriveCommand extends CharacterizeDriveCommand {
    private final double stepLength;
    private final double stepHeight;

    public StepCharacterizeDriveCommand(double stepLength, double stepHeight, SwerveDriveSubsystem driveSubsystem) {
        super(driveSubsystem);

        this.stepLength = stepLength;
        this.stepHeight = stepHeight;
    }

    @Override
    protected double getVoltage(double currentTime) {
        int stepIndex = ((int) (currentTime / stepLength)) + 1;
        return stepIndex * stepHeight;
    }
}
