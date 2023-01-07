package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.HashMap;

public class RaiderSwerveAutoBuilder extends BaseAutoBuilder {
    private final SwerveDriveSubsystem driveSubsystem;

    public RaiderSwerveAutoBuilder(HashMap<String, Command> eventMap, SwerveDriveSubsystem driveSubsystem) {
        super(driveSubsystem::getPose, driveSubsystem::resetOdometry, eventMap, DrivetrainType.HOLONOMIC);
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new FollowPathCommand(trajectory, driveSubsystem);
    }
}
