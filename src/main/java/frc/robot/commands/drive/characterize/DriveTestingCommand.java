package frc.robot.commands.drive.characterize;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveTestingCommand extends CommandBase {
    private final double rampRateMetersPerSecond;
    private final boolean useNextStates;
    private final SwerveDriveSubsystem driveSubsystem;
    private double startTime;

    public DriveTestingCommand(
            double rampRateMetersPerSecond, boolean useNextStates, SwerveDriveSubsystem driveSubsystem) {
        this.rampRateMetersPerSecond = rampRateMetersPerSecond;
        this.useNextStates = useNextStates;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        SwerveModuleState[] states = new SwerveModuleState[DriveTrainConstants.NUM_MODULES];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(rampRateMetersPerSecond * elapsedTime, Rotation2d.fromDegrees(0.0));
        }

        if (useNextStates) {
            SwerveModuleState[] nextStates = new SwerveModuleState[DriveTrainConstants.NUM_MODULES];
            for (int i = 0; i < states.length; i++) {
                nextStates[i] = new SwerveModuleState(
                        rampRateMetersPerSecond * (elapsedTime + Constants.DT), Rotation2d.fromDegrees(0.0));
            }
            driveSubsystem.setRawStates(true, false, states, nextStates);
        } else {
            driveSubsystem.setRawStates(true, false, states);
        }
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
