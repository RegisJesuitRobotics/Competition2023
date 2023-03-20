package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SimpleVelocityCommand extends CommandBase {
    private final ChassisSpeeds desiredSpeeds;
    private final SwerveDriveSubsystem driveSubsystem;

    public SimpleVelocityCommand(ChassisSpeeds desiredSpeeds, SwerveDriveSubsystem driveSubsystem) {
        this.desiredSpeeds = desiredSpeeds;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.setChassisSpeeds(desiredSpeeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
