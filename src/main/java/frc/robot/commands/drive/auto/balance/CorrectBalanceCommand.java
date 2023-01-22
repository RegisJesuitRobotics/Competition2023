package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;

public class CorrectBalanceCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;

    private final TunableTelemetryPIDController balanceController =
            new TunableTelemetryPIDController("/drive/balanceController", AutoConstants.AUTO_BALANCE_PID_GAINS);

    public CorrectBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        balanceController.setTolerance(1.5);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        balanceController.reset();
        balanceController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        double desiredVelocity = MathUtil.clamp(
                balanceController.calculate(driveSubsystem.getPitch()),
                -AutoConstants.MAX_AUTO_BALANCE_VELOCITY,
                AutoConstants.MAX_AUTO_BALANCE_VELOCITY);
        driveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, desiredVelocity, 0), false);
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
