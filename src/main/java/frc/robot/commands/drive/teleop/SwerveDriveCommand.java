package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.utils.RaiderMathUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveDriveCommand extends CommandBase {
    private final Supplier<Translation2d> translationSupplier;
    private final DoubleSupplier omegaRadiansSecondSupplier;
    private final BooleanSupplier isSnapSupplier;
    private final DoubleSupplier snapAngleSupplier;
    private final BooleanSupplier isFieldRelativeSupplier;

    private final TunableTelemetryProfiledPIDController snapPIDController = new TunableTelemetryProfiledPIDController(
            "drive/snapController",
            AutoConstants.SNAP_ANGULAR_POSITION_PID_GAINS,
            AutoConstants.SNAP_ANGULAR_POSITION_TRAPEZOIDAL_GAINS);
    private final SwerveDriveSubsystem driveSubsystem;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private boolean isSnapping = false;

    public SwerveDriveCommand(
            Supplier<Translation2d> translationSupplier,
            DoubleSupplier omegaRadiansSecondSupplier,
            BooleanSupplier isSnapSupplier,
            DoubleSupplier snapAngleSupplier,
            BooleanSupplier isFieldRelativeSupplier,
            SwerveDriveSubsystem driveSubsystem) {
        this.translationSupplier = translationSupplier;
        this.omegaRadiansSecondSupplier = omegaRadiansSecondSupplier;
        this.isSnapSupplier = isSnapSupplier;
        this.snapAngleSupplier = snapAngleSupplier;
        this.isFieldRelativeSupplier = isFieldRelativeSupplier;

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        isSnapping = false;
    }

    @Override
    public void execute() {
        Robot.startWNode("DriveExecute");
        boolean isFieldRelative = isFieldRelativeSupplier.getAsBoolean();
        Translation2d translation = translationSupplier.get();

        Rotation2d currentHeading = driveSubsystem.getPose().getRotation();
        double omega;
        // Snap does not work if we are field relative
        if (isSnapSupplier.getAsBoolean() && isFieldRelative) {
            double currentOmegaRadiansSecond = driveSubsystem.getCurrentChassisSpeeds().omegaRadiansPerSecond;
            if (!isSnapping) {
                snapPIDController.reset(currentHeading.getRadians(), currentOmegaRadiansSecond);
                isSnapping = true;
            }
            omega = snapPIDController.calculate(currentHeading.getRadians(), snapAngleSupplier.getAsDouble())
                    + snapPIDController.getSetpoint().velocity;
        } else {
            omega = omegaRadiansSecondSupplier.getAsDouble();
            isSnapping = false;
        }

        ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), omega);

        if (isFieldRelative) {
            nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(nextChassisSpeeds, driveSubsystem.getPose().getRotation());
        }

        if (RaiderMathUtils.isChassisSpeedsZero(
                chassisSpeeds,
                DriveTrainConstants.TELEOP_MINIMUM_VELOCITY_METERS_SECOND,
                DriveTrainConstants.TELEOP_MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND)) {
            driveSubsystem.stopMovement();
        } else {
            // FIXME
            //            driveSubsystem.setChassisSpeeds(chassisSpeeds, nextChassisSpeeds, true);
            driveSubsystem.setChassisSpeeds(chassisSpeeds, false);
        }

        chassisSpeeds = nextChassisSpeeds;
        Robot.endWNode();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
