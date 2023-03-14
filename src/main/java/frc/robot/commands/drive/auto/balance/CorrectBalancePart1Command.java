package frc.robot.commands.drive.auto.balance;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class CorrectBalancePart1Command extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;

    private boolean hasPassedZero = false;
    private double startAngle = 0.0;

    public CorrectBalancePart1Command(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    public static double getAngle(Rotation2d yaw, double pitch, double roll) {
        return yaw.getSin() * pitch + yaw.getCos() * roll;
    }

    @Override
    public void initialize() {
        hasPassedZero = false;
        startAngle = getAngle(
                driveSubsystem.getPose().getRotation(),
                driveSubsystem.getPitchRadians(),
                driveSubsystem.getRollRadians());
    }

    @Override
    public void execute() {
        double currentAngle = getAngle(
                driveSubsystem.getPose().getRotation(),
                driveSubsystem.getPitchRadians(),
                driveSubsystem.getRollRadians());
        if (!hasPassedZero) {
            if (Math.signum(currentAngle) != Math.signum(startAngle)) {
                hasPassedZero = true;
            } else {
                driveSubsystem.setChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                Math.signum(currentAngle) * -AutoConstants.AUTO_BALANCE_SPEED_METERS_SECOND,
                                0,
                                0,
                                driveSubsystem.getPose().getRotation()),
                        false);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return hasPassedZero;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }
}
