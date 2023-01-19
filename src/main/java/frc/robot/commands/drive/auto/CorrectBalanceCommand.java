package frc.robot.commands.drive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import static frc.robot.Constants.AutoConstants.AUTO_BALANCE_PID_GAINS;


public class CorrectBalanceCommand extends CommandBase {
    private SwerveDriveSubsystem driveSubsystem;
//TODO: maybe make this profiled and create constant
    private PIDController pidController = new PIDController(0, 0, 0);


    public CorrectBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
