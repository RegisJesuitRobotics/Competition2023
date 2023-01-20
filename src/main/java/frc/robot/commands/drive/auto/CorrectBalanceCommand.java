package frc.robot.commands.drive.auto;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
        double xTranslation = pidController.calculate(driveSubsystem.getPitch(), 0);
        ChassisSpeeds speeds = new ChassisSpeeds(xTranslation, 0, 0);


        driveSubsystem.setChassisSpeeds(speeds, true);

    }

    private float getPitch(){
        return gyro.getPitch();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getPitch()) < 2;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
