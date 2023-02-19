package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.characterize.*;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.Map;

public class Autos {
    private final HashMap<String, Command> autoMap = new HashMap<>();
    private final RaiderSwerveAutoBuilder autoBuilder;

    public Autos(SwerveDriveSubsystem driveSubsystem) {
        HashMap<String, Command> eventMap =
                new HashMap<>(Map.ofEntries(Map.entry("LockModules", new LockModulesCommand(driveSubsystem))));

        autoBuilder = new RaiderSwerveAutoBuilder(eventMap, driveSubsystem);

        addPPAuto("New Path");
        addPPAuto("Testing");
        addPPAuto("Straight Rotation");
        if (MiscConstants.TUNING_MODE) {
            addAuto("SteerTesting", new SteerTestingCommand(driveSubsystem));
            addAuto("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
            addAuto("GreaseGears", new GreaseGearsCommand(driveSubsystem));
            addAuto("SysIDLogger", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
        }
    }

    private void addPPAuto(String name) {
        addAuto(name, autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, AutoConstants.PATH_CONSTRAINTS)));
    }

    private void addAuto(String name, Command command) {
        autoMap.put(name, command);
    }

    public Map<String, Command> getAutos() {
        return autoMap;
    }
}
