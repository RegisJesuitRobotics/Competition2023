package frc.robot.commands.drive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunablePIDGains;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;

public class AutoBalanceCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final TunableTelemetryPIDController pidController = new TunableTelemetryPIDController("/autobalance", new TunablePIDGains("/autobalance/gains", 0.01, 0.0, 0.0, true));

    private double startTime = 0.0;

    public AutoBalanceCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        // FIXME: what if roll should not be reset? use april tags or smthing
        pidController.setSetpoint(0.0);
        driveSubsystem.resetPitch();
    }

    @Override
    public void execute() {
        double output;
        if (Timer.getFPGATimestamp() - startTime < 3.0) {
            output = 0.5;
        } else {
            output = pidController.calculate(driveSubsystem.getPitch());
            output += Math.signum(output) * 0.2;
            SmartDashboard.putNumber("DSF", output);
        }
        driveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, output, 0.0), true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    boolean isLevel = false;
    double levelStartTime;
    @Override
    public boolean isFinished() {
        if (isLevel) {
            if (Math.abs(driveSubsystem.getPitch()) > 2.5) {
                isLevel = false;
            }
        } else {
            if (Math.abs(driveSubsystem.getPitch()) < 2.5) {
                isLevel = true;
                levelStartTime = Timer.getFPGATimestamp();
            }
        }
        return (isLevel && Timer.getFPGATimestamp() - levelStartTime > 0.5) && Timer.getFPGATimestamp() - startTime > 1.0;
    }
}
