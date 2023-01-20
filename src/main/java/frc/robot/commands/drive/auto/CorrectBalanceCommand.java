package frc.robot.commands.drive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class CorrectBalanceCommand extends CommandBase {
    private SwerveDriveSubsystem driveSubsystem;
    // TODO: maybe make this profiled and create constant
    private PIDController pidController = new PIDController(0, 0, 0);

    public CorrectBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xTranslation = pidController.calculate(driveSubsystem.getPitch(), 0);
        ChassisSpeeds speeds = new ChassisSpeeds(xTranslation, 0, 0);

        driveSubsystem.setChassisSpeeds(speeds, true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveSubsystem.getPitch()) < 2;
    }

    @Override
    public void end(boolean interrupted) {}
}
