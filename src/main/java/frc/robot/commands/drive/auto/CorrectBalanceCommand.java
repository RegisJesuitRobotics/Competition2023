package frc.robot.commands.drive.auto;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;

public class CorrectBalanceCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;

    // TODO: Possibly use a profiled PID controller
    private final TunableTelemetryPIDController balanceController =
            new TunableTelemetryPIDController("/drive/balanceController", AutoConstants.AUTO_BALANCE_PID_GAINS);

    public CorrectBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        balanceController.setTolerance(2.0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        balanceController.reset();
        balanceController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        driveSubsystem.setChassisSpeeds(
                new ChassisSpeeds(0, balanceController.calculate(driveSubsystem.getPitch()), 0), false);
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
