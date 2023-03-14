package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * This command puts the swerve modules in an X pattern making it hard to be moved when bumped. It
 * does not finish
 */
public class LockModulesParallelCommand extends CommandBase {
    private static final Rotation2d zeroDegrees = Rotation2d.fromDegrees(0);
    private static final SwerveModuleState[] states = new SwerveModuleState[] {
        new SwerveModuleState(0.0, zeroDegrees), new SwerveModuleState(0.0, zeroDegrees.unaryMinus()),
        new SwerveModuleState(0.0, zeroDegrees.unaryMinus()), new SwerveModuleState(0.0, zeroDegrees)
    };

    private final SwerveDriveSubsystem driveSubsystem;

    public LockModulesParallelCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.setRawStates(true, true, states);
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
