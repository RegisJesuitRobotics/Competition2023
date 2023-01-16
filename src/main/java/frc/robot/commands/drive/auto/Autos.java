package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
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
        addAuto("QuasistaticCharacterization", new QuasistaticCharacterizeDriveCommand(0.4, driveSubsystem));
        addAuto("DynamicCharacterization", new DynamicCharacterizeDriveCommand(8.0, driveSubsystem));
        addAuto("StepCharacterization", new StepCharacterizeDriveCommand(3.0, 2.0, driveSubsystem));
        addAuto("SteerTesting", new SteerTestingCommand(driveSubsystem));
        addAuto("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
        addAuto("GreaseGears", new GreaseGearsCommand(driveSubsystem));
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
