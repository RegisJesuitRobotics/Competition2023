package frc.robot.commands.drive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import java.util.function.Supplier;

public class SimpleToPointCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<Pose2d> desiredPoseSupplier;
    private final TunableTelemetryProfiledPIDController translationController =
            new TunableTelemetryProfiledPIDController(
                    "/simpleToPoint/translationController",
                    AutoConstants.TRANSLATION_POSITION_GAINS,
                    AutoConstants.TRANSLATION_POSITION_TRAPEZOIDAL_GAINS);
    private final DoubleTelemetryEntry translationErrorEntry =
            new DoubleTelemetryEntry("/simpleToPoint/translationError", MiscConstants.TUNING_MODE);
    private final TunableTelemetryProfiledPIDController rotationController = new TunableTelemetryProfiledPIDController(
            "/simpleToPoint/rotationController",
            AutoConstants.ANGULAR_POSITION_PID_GAINS,
            AutoConstants.ANGULAR_POSITION_TRAPEZOIDAL_GAINS);

    private Pose2d currentDesiredPose = new Pose2d();

    public SimpleToPointCommand(Supplier<Pose2d> desiredPoseSupplier, SwerveDriveSubsystem driveSubsystem) {
        this.desiredPoseSupplier = desiredPoseSupplier;
        this.driveSubsystem = driveSubsystem;

        translationController.setTolerance(0.05);
        rotationController.setTolerance(Units.degreesToRadians(3.0));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentDesiredPose = desiredPoseSupplier.get();
        // Aim for zero error in translation
        translationController.setGoal(0.0);
        rotationController.setGoal(currentDesiredPose.getRotation().getRadians());

        Translation2d translationError =
                driveSubsystem.getPose().getTranslation().minus(currentDesiredPose.getTranslation());
        ChassisSpeeds speeds = driveSubsystem.getCurrentChassisSpeeds();
        translationController.reset(
                translationError.getNorm(), Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        rotationController.reset(driveSubsystem.getPose().getRotation().getRadians(), speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Translation2d translationError =
                driveSubsystem.getPose().getTranslation().minus(currentDesiredPose.getTranslation());
        translationErrorEntry.append(translationError.getNorm());
        double translationFeedback = translationController.calculate(translationError.getNorm());
        double translationFeedforward = translationController.getSetpoint().velocity;
        Translation2d translationVelocity =
                new Translation2d(translationFeedback + translationFeedforward, translationError.getAngle());

        double angularVelocity = rotationController.calculate(
                driveSubsystem.getPose().getRotation().getRadians());

        driveSubsystem.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationVelocity.getX(),
                        translationVelocity.getY(),
                        angularVelocity,
                        driveSubsystem.getPose().getRotation()),
                false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return translationController.atGoal() && rotationController.atGoal();
    }
}
