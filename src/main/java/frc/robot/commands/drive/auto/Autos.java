package frc.robot.commands.drive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.characterize.*;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.Map;

public class Autos {
    private final HashMap<String, Command> autoMap = new HashMap<>();

    public Autos(SwerveDriveSubsystem driveSubsystem) {
        addAuto("SysIDCharacterization", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
        addAuto("SteerTesting", new SteerTestingCommand(driveSubsystem));
        addAuto("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
        addAuto("GreaseGears", new GreaseGearsCommand(driveSubsystem));
    }

    private void addAuto(String name, Command command) {
        autoMap.put(name, command);
    }

    public Map<String, Command> getAutos() {
        return autoMap;
    }
}
