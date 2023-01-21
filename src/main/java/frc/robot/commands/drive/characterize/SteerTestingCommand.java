package frc.robot.commands.drive.characterize;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableDouble;

public class SteerTestingCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final TunableDouble desiredSteer = new TunableDouble("/char/desiredSteerTesting", 0.0, true);

    public SteerTestingCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d angle = Rotation2d.fromDegrees(desiredSteer.get());

        SwerveModuleState[] states = new SwerveModuleState[DriveTrainConstants.NUM_MODULES];

        for (int i = 0; i < DriveTrainConstants.NUM_MODULES; i++) {
            states[i] = new SwerveModuleState(0.0, angle);
        }

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
