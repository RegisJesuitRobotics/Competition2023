package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class CorrectBalanceCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;

    private final BangBangController balanceController =
            new BangBangController(AutoConstants.AUTO_BALANCE_TOLERANCE_RADIANS);

    public CorrectBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        balanceController.setTolerance(1.5);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        balanceController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        driveSubsystem.setChassisSpeeds(
                new ChassisSpeeds(
                        0,
                        balanceController.calculate(driveSubsystem.getPitch())
                                * AutoConstants.AUTO_BALANCE_SPEED_METERS_SECOND,
                        0),
                false);
    }

    @Override
    public boolean isFinished() {
        return balanceController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }
}
