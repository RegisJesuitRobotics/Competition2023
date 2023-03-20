package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.filter.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderMathUtils;

public class NewCorrectBalancePart1Command extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Debouncer atZeroDebouncer = new Debouncer(1.0);

    private double startAngle = 0.0;

    public NewCorrectBalancePart1Command(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        startAngle = driveSubsystem.getFieldCentricRoll();
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
        return Math.signum(driveSubsystem.getFieldCentricRoll()) != Math.signum(driveSubsystem.getRollVelocity())
                || atZeroDebouncer.calculate(RaiderMathUtils.inAbsRange(driveSubsystem.getFieldCentricRoll(), 0.05));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }
}
