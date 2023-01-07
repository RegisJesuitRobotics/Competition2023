package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.auto.Autos;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableDouble;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ListenableSendableChooser;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();

    private final CommandXboxPlaystationController driverController = new CommandXboxPlaystationController(0);
    private final TeleopControlsStateManager teleopControlsStateManager = new TeleopControlsStateManager();

    private final ListenableSendableChooser<Command> driveCommandChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<Command> autoCommandChooser = new ListenableSendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    public RobotContainer() {
        configureButtonBindings();
        configureAutos();

        Shuffleboard.getTab("UtilsRaw").add(CommandScheduler.getInstance());
    }

    private void configureAutos() {
        if (MiscConstants.TUNING_MODE) {
            PathPlannerServer.startServer(5811);
        }

        autoCommandChooser.setDefaultOption("Nothing", null);
        Autos autos = new Autos(driveSubsystem);
        for (Entry<String, Command> auto : autos.getAutos().entrySet()) {
            autoCommandChooser.addOption(auto.getKey(), auto.getValue());
        }

        new Trigger(autoCommandChooser::hasNewValue)
                .onTrue(Commands.runOnce(() -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .ignoringDisable(true)
                        .withName("Auto Alert Checker"));

        Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        TunableDouble maxTranslationSpeedPercent = new TunableDouble("speed/maxTranslation", 0.9, true);
        TunableDouble maxMaxAngularSpeedPercent = new TunableDouble("speed/maxAngular", 0.75, true);

        DoubleSupplier maxTranslationalSpeedSuppler =
                () -> maxTranslationSpeedPercent.get() * DriveTrainConstants.MAX_VELOCITY_METERS_SECOND;
        DoubleSupplier maxAngularSpeedSupplier =
                () -> maxMaxAngularSpeedPercent.get() * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND;

        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative & absolute control but use robot centric when holding button)",
                new SwerveDriveCommand(
                        () -> new Translation2d(-driverController.getLeftY(), -driverController.getLeftX())
                                .times(maxTranslationalSpeedSuppler.getAsDouble()),
                        () -> -driverController.getRightX(),
                        driverController.leftBumper().negate(),
                        () -> Math.atan2(driverController.getRightY(), driverController.getRightX()),
                        driverController.rightBumper().negate(),
                        driveSubsystem));
        driveCommandChooser.addOption(
                "Robot Orientated",
                new SwerveDriveCommand(
                        () -> new Translation2d(-driverController.getLeftY(), -driverController.getLeftX())
                                .times(maxTranslationalSpeedSuppler.getAsDouble()),
                        () -> -driverController.getRightX() * maxAngularSpeedSupplier.getAsDouble(),
                        () -> false,
                        () -> 0.0,
                        () -> false,
                        driveSubsystem));

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);

        evaluateDriveStyle(driveCommandChooser.getSelected());
        new Trigger(driveCommandChooser::hasNewValue)
                .onTrue(Commands.runOnce(() -> evaluateDriveStyle(driveCommandChooser.getSelected()))
                        .ignoringDisable(true)
                        .withName("Drive Style Checker"));

        driverController
                .circle()
                .onTrue(Commands.runOnce(driveSubsystem::resetOdometry)
                        .ignoringDisable(true)
                        .withName("Reset Odometry"));
        driverController
                .povUp()
                .whileTrue(new LockModulesCommand(driveSubsystem).repeatedly().withName("Lock Modules"));

        driverController
                .square()
                .debounce(0.5)
                .onTrue(new FollowPathCommand(
                                () -> {
                                    Pose2d currentPose = driveSubsystem.getPose();
                                    Pose2d targetPose = new Pose2d();
                                    Translation2d translation =
                                            currentPose.minus(targetPose).getTranslation();
                                    return PathPlanner.generatePath(
                                            AutoConstants.PATH_CONSTRAINTS,
                                            new PathPoint(
                                                    currentPose.getTranslation(),
                                                    new Rotation2d(-translation.getX(), -translation.getY()),
                                                    currentPose.getRotation()),
                                            new PathPoint(
                                                    new Translation2d(0, 0),
                                                    new Rotation2d(-translation.getX(), -translation.getY()),
                                                    new Rotation2d(0)));
                                },
                                driveSubsystem)
                        .until(driverController.rightBumper())
                        .withName("To (0, 0) Follow Path"));
    }

    private void evaluateDriveStyle(Command newCommand) {
        Command oldCommand = driveSubsystem.getDefaultCommand();

        // Check if they are the same
        // we use the == operator instead of Command#equals() because we want to know if
        // it is the exact same object in memory
        if (newCommand == oldCommand) {
            return;
        }
        driveSubsystem.setDefaultCommand(newCommand);
        if (oldCommand != null) {
            // We have to cancel the command so the new default one will run
            oldCommand.cancel();
        }
    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
