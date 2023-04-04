package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderMathUtils;

public class CorrectBalancePart1Command extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;

    private final Debouncer atZeroDebouncer = new Debouncer(1.0);
    private double startTime = 0.0;

    public CorrectBalancePart1Command(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        atZeroDebouncer.calculate(false);
    }

    @Override
    public void execute() {
        double currentAngle = driveSubsystem.getFieldCentricRoll();
        driveSubsystem.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        Math.signum(currentAngle) * -AutoConstants.AUTO_BALANCE_SPEED_METERS_SECOND,
                        0,
                        0,
                        driveSubsystem.getPose().getRotation()),
                false);
    }

    @Override
    public boolean isFinished() {
        return ((Math.signum(driveSubsystem.getFieldCentricRoll())
                                        != Math.signum(driveSubsystem.getFieldCentricRollVelocity())
                                && !RaiderMathUtils.inAbsRange(driveSubsystem.getFieldCentricRollVelocity(), 0.05))
                        || atZeroDebouncer.calculate(
                                RaiderMathUtils.inAbsRange(driveSubsystem.getFieldCentricRoll(), 0.05)))
                && (Timer.getFPGATimestamp() - startTime > 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }
}
