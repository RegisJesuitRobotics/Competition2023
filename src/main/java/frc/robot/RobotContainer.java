package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.characterize.DriveTestingCommand;
import frc.robot.commands.drive.characterize.DriveTrainSysIDCompatibleLoggerCommand;
import frc.robot.commands.drive.characterize.SteerTestingCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.tunable.TunableDouble;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.RaiderMathUtils;
import frc.robot.utils.VectorRateLimiter;
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
        ConfigurablePaths paths = new ConfigurablePaths(driveSubsystem);
        autoCommandChooser.setDefaultOption("GeneratedAuto", new ProxyCommand(paths::generatePath));
        autoCommandChooser.addOption("Nothing", null);

        if (MiscConstants.TUNING_MODE) {
            autoCommandChooser.addOption("SysIDLogger", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
            autoCommandChooser.addOption("GreaseGears", new GreaseGearsCommand(driveSubsystem));
            autoCommandChooser.addOption("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
            autoCommandChooser.addOption("SteerTesting", new SteerTestingCommand(driveSubsystem));
        }

        new Trigger(autoCommandChooser::hasNewValue)
                .onTrue(Commands.runOnce(() -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .ignoringDisable(true)
                        .withName("Auto Alert Checker"));

        SendableTelemetryManager.getInstance().addSendable("/autoChooser/AutoChooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        TunableDouble maxTranslationSpeedPercent = new TunableDouble("/speed/maxTranslation", 0.9, true);
        TunableDouble maxMaxAngularSpeedPercent = new TunableDouble("/speed/maxAngular", 0.5, true);

        DoubleSupplier maxTranslationalSpeedSuppler = () -> maxTranslationSpeedPercent.get()
                * DriveTrainConstants.MAX_VELOCITY_METERS_SECOND
                * (driverController.leftBumper().getAsBoolean() ? 0.5 : 1);
        DoubleSupplier maxAngularSpeedSupplier =
                () -> maxMaxAngularSpeedPercent.get() * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND;

        SlewRateLimiter rotationLimiter =
                new SlewRateLimiter(TeleopConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED);
        VectorRateLimiter vectorRateLimiter =
                new VectorRateLimiter(TeleopConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED);
        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative & absolute control but use robot centric when holding button)",
                new SwerveDriveCommand(
                        () -> vectorRateLimiter.calculate(new Translation2d(
                                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftY()),
                                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftX()))
                                .times(maxTranslationalSpeedSuppler.getAsDouble())),
                        () -> rotationLimiter.calculate(
                                RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
                                        * maxAngularSpeedSupplier.getAsDouble()),
                        driverController
                                .triangle()
                                .or(driverController.circle())
                                .or(driverController.x())
                                .or(driverController.square()),
                        () -> {
                            if (driverController.square().getAsBoolean()) return Math.PI / 2;
                            if (driverController.x().getAsBoolean()) return Math.PI;
                            if (driverController.circle().getAsBoolean()) return -Math.PI / 2;
                            return 0.0;
                        },
                        driverController.rightBumper().negate(),
                        driveSubsystem));

        driveCommandChooser.addOption(
                "Robot Orientated",
                new SwerveDriveCommand(
                        () -> new Translation2d(
                                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftY()),
                                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftX()))
                                .times(maxTranslationalSpeedSuppler.getAsDouble()),
                        () -> RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
                                * maxAngularSpeedSupplier.getAsDouble(),
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
                .options()
                .onTrue(Commands.runOnce(driveSubsystem::resetOdometry)
                        .ignoringDisable(true)
                        .withName("Reset Odometry"));

        driverController
                .povUp()
                .whileTrue(new LockModulesCommand(driveSubsystem).repeatedly().withName("Lock Modules"));
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
