package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.Map;

public class RaiderAutoBuilder extends BaseAutoBuilder {
    private final SwerveDriveSubsystem driveSubsystem;

    public RaiderAutoBuilder(Map<String, Command> eventMap, SwerveDriveSubsystem driveSubsystem) {
        // Do not reset pose, april tags will do it
        super(driveSubsystem::getPose, (Pose2d) -> {}, eventMap, DrivetrainType.HOLONOMIC);

        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new FollowPathCommand(trajectory, driveSubsystem);
    }
}
