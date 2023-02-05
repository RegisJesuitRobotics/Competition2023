package frc.robot.commands.drive.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ConfigurablePaths;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.characterize.*;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.Map;

public class Autos {
    private final HashMap<String, Command> autoMap = new HashMap<>();

    public Autos(SwerveDriveSubsystem driveSubsystem) {
        ConfigurablePaths paths = new ConfigurablePaths(driveSubsystem);
        if (MiscConstants.TUNING_MODE) {
            addAuto("SysIDLogger", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
            addAuto("GreaseGears", new GreaseGearsCommand(driveSubsystem));
            addAuto("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
            addAuto("SteerTesting", new SteerTestingCommand(driveSubsystem));
        }
        addAuto("GeneratePath", new ProxyCommand(paths::generatePath));
    }

    private void addAuto(String name, Command command) {
        autoMap.put(name, command);
    }

    public Map<String, Command> getAutos() {
        return autoMap;
    }
}
