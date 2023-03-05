package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.FieldConstants.Grids;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.HomeCommandFactory;
import frc.robot.commands.PositionClawCommand;
import frc.robot.commands.drive.GreaseGearsCommand;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.characterize.DriveTestingCommand;
import frc.robot.commands.drive.characterize.DriveTrainSysIDCompatibleLoggerCommand;
import frc.robot.commands.drive.characterize.SteerTestingCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.commands.flipper.FullyToggleFlipperCommand;
import frc.robot.commands.led.LEDCommandFactory;
import frc.robot.commands.led.LEDStateMachineCommand;
import frc.robot.commands.led.LEDStateMachineCommand.LEDState;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.intake.FlipperSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.photon.PhotonSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.tunable.gains.TunableDouble;
import frc.robot.utils.*;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.led.AlternatePattern;
import frc.robot.utils.led.RandomColorsPattern;
import frc.robot.utils.led.SlidePattern;
import frc.robot.utils.led.SolidPattern;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final PhotonSubsystem cameraWrapperSubsystem = new PhotonSubsystem();
    private final SwerveDriveSubsystem driveSubsystem =
            new SwerveDriveSubsystem(cameraWrapperSubsystem::getEstimatedGlobalPose);
    private final FlipperSubsystem flipperSubsystem = new FlipperSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final LiftSubsystem liftSubsystem = new LiftSubsystem();
    private final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final CommandNintendoSwitchController driverController = new CommandNintendoSwitchController(0);
    private final CommandXboxPlaystationController operatorController = new CommandXboxPlaystationController(1);
    private final TeleopControlsStateManager teleopControlsStateManager = new TeleopControlsStateManager();

    private final ListenableSendableChooser<Command> driveCommandChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<Command> autoCommandChooser = new ListenableSendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    private final IntegerEntry gridEntry = NetworkTableInstance.getDefault()
            .getIntegerTopic("/toLog/autoScore/grid")
            .getEntry(0);

    public RobotContainer() {
        configureDriverBindings();
        configureOperatorBindings();
        configureAutos();

        Shuffleboard.getTab("UtilsRaw").add(CommandScheduler.getInstance());
        liftSubsystem.setDefaultCommand(Commands.run(liftSubsystem::stopMovement, liftSubsystem));
        extensionSubsystem.setDefaultCommand(Commands.run(extensionSubsystem::stopMovement, extensionSubsystem));

        List<LEDState> ledStates = List.of(
                // Party mode on flip is #1 priority
                new LEDState(
                        () -> Math.abs(driveSubsystem.getRoll()) > 50.0 || Math.abs(driveSubsystem.getPitch()) > 50.0,
                        new AlternatePattern(
                                2.0 / 5.0, new RandomColorsPattern(2.0 / 5.0), new SolidPattern(Color.kBlack)),
                        0),
                // Red blink if we have any faults
                new LEDState(
                        () -> Alert.getDefaultGroup().hasAnyErrors(),
                        new AlternatePattern(2.0, Color.kRed, Color.kBlack),
                        1),
                // Orange if we are to close to the grid to bring arm down
                new LEDState(
                        () -> RaiderUtils.flipIfShould(driveSubsystem.getPose()).getX() < 2.4
                                && LiftExtensionKinematics.liftExtensionPositionToClawPosition(
                                                        liftSubsystem.getArmAngle(), extensionSubsystem.getPosition())
                                                .getY()
                                        < Grids.midCubeZ,
                        new SolidPattern(Color.kOrange),
                        2),
                // Default disabled pattern
                new LEDState(
                        DriverStation::isDisabled,
                        new AlternatePattern(
                                8.0,
                                new SlidePattern(8 / 2.0, Color.kDarkRed, Color.kWhite),
                                new SlidePattern(8 / 2.0, Color.kWhite, Color.kDarkRed)),
                        5));

        ledSubsystem.setDefaultCommand(
                new LEDStateMachineCommand(new SolidPattern(Color.kBlack), ledStates, ledSubsystem));
    }

    private void configureAutos() {
        ConfigurablePaths paths = new ConfigurablePaths(
                driveSubsystem, liftSubsystem, extensionSubsystem, clawSubsystem, flipperSubsystem);
        SendableTelemetryManager.getInstance()
                .addSendable(
                        "/autoChooser/generatePath",
                        RaiderCommands.runOnceAllowDisable(paths::generatePath).withName("Generate Path"));

        autoCommandChooser.addOption("Nothing", null);
        autoCommandChooser.setDefaultOption(
                "Only Home",
                Commands.parallel(
                        HomeCommandFactory.homeLiftCommand(liftSubsystem),
                        HomeCommandFactory.homeExtensionCommand(extensionSubsystem)));
        autoCommandChooser.addOption("GeneratedAuto", new ProxyCommand(paths::getCurrentCommandAndUpdateIfNeeded));

        if (MiscConstants.TUNING_MODE) {
            autoCommandChooser.addOption("SysIDLogger", new DriveTrainSysIDCompatibleLoggerCommand(driveSubsystem));
            autoCommandChooser.addOption("GreaseGears", new GreaseGearsCommand(driveSubsystem));
            autoCommandChooser.addOption("DriveTestingCommand", new DriveTestingCommand(1.0, true, driveSubsystem));
            autoCommandChooser.addOption("SteerTesting", new SteerTestingCommand(driveSubsystem));
        }

        new Trigger(autoCommandChooser::hasNewValue)
                .onTrue(RaiderCommands.runOnceAllowDisable(
                                () -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .withName("Auto Alert Checker"));

        SendableTelemetryManager.getInstance().addSendable("/autoChooser/AutoChooser", autoCommandChooser);
    }

    private void configureDriverBindings() {
        configureDriving();

        driverController.circle().onTrue(RaiderCommands.runOnceAllowDisable(driveSubsystem::zeroHeading));
        driverController.minus().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());
        // driverController.plus().whileTrue(new LockModulesParallelCommand(driveSubsystem).repeatedly());
        driverController
                .leftStick()
                .onTrue(new PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem));

        driverController.rightStick().onTrue(Commands.runOnce(clawSubsystem::toggleClawState, clawSubsystem));
        driverController.leftTrigger().whileTrue(new FullyToggleFlipperCommand(flipperSubsystem));
        driverController
                .rightTrigger()
                .onTrue(new PositionClawCommand(AutoScoreConstants.CARRY, liftSubsystem, extensionSubsystem));

        new Trigger(() -> DriverStation.isTeleop() && Timer.getMatchTime() < 0.5)
                .onTrue(new LockModulesCommand(driveSubsystem).withTimeout(1.0));

        Trigger driverTakeControl = new Trigger(() -> !RaiderMathUtils.inAbsRange(
                                driverController.getLeftX(), TeleopConstants.DRIVER_TAKE_CONTROL_THRESHOLD)
                        || !RaiderMathUtils.inAbsRange(
                                driverController.getLeftY(), TeleopConstants.DRIVER_TAKE_CONTROL_THRESHOLD)
                        || !RaiderMathUtils.inAbsRange(
                                driverController.getRightX(), TeleopConstants.DRIVER_TAKE_CONTROL_THRESHOLD)
                        || !RaiderMathUtils.inAbsRange(
                                driverController.getRightY(), TeleopConstants.DRIVER_TAKE_CONTROL_THRESHOLD))
                .debounce(0.1);

        IntSupplier gridSupplier = () -> {
            int grid = (int) gridEntry.get();
            if (RaiderUtils.shouldFlip()) {
                return grid;
            }
            return 8 - grid;
        };
        Trigger inAllowedArea = new Trigger(() -> AutoScoreConstants.ALLOWED_SCORING_AREA.isPointInside(
                RaiderUtils.flipIfShould(driveSubsystem.getPose()).getTranslation()));

        driverController
                .povUp()
                .debounce(0.1)
                .onTrue(RaiderCommands.ifCondition(inAllowedArea)
                        .then(new AutoScoreCommand(gridSupplier, driveSubsystem).until(driverTakeControl))
                        .otherwise(rumbleDriverControllerCommand()));
    }

    private void configureOperatorBindings() {
        operatorController
                .povUp()
                .whileTrue(new PositionClawCommand(AutoScoreConstants.HIGH, liftSubsystem, extensionSubsystem)
                        .andThen(rumbleOperatorControllerCommand()));
        operatorController
                .povRight()
                .whileTrue(new PositionClawCommand(AutoScoreConstants.MID, liftSubsystem, extensionSubsystem)
                        .andThen(rumbleOperatorControllerCommand()));
        operatorController
                .povDown()
                .whileTrue(new PositionClawCommand(AutoScoreConstants.LOW, liftSubsystem, extensionSubsystem)
                        .andThen(rumbleOperatorControllerCommand()));
        operatorController
                .povLeft()
                .whileTrue(new PositionClawCommand(
                                AutoScoreConstants.SUBSTATION_LOCATION, liftSubsystem, extensionSubsystem)
                        .andThen(rumbleOperatorControllerCommand()));

        operatorController
                .x()
                .whileTrue(new PositionClawCommand(AutoScoreConstants.STOW, liftSubsystem, extensionSubsystem)
                        .andThen(rumbleOperatorControllerCommand()));

        // Lift override, ranges from 0 to 6 volts
        new Trigger(() -> !RaiderMathUtils.inAbsRange(operatorController.getLeftY(), 0.1))
                .whileTrue(Commands.runEnd(
                        () -> liftSubsystem.setVoltage(
                                MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * -6.0),
                        () -> liftSubsystem.setVoltage(0.0),
                        liftSubsystem));

        // Extension override, ranges from 0 - 6 volts
        new Trigger(() -> !RaiderMathUtils.inAbsRange(operatorController.getRightX(), 0.1))
                .whileTrue(Commands.runEnd(
                        () -> extensionSubsystem.setVoltage(
                                MathUtil.applyDeadband(operatorController.getRightX(), 0.1) * -6.0),
                        () -> extensionSubsystem.setVoltage(0.0),
                        extensionSubsystem));

        operatorController
                .share()
                .onTrue(HomeCommandFactory.homeLiftCommand(liftSubsystem).andThen(rumbleOperatorControllerCommand()));
        operatorController
                .options()
                .onTrue(HomeCommandFactory.homeExtensionCommand(extensionSubsystem)
                        .andThen(rumbleOperatorControllerCommand()));

        operatorController.leftTrigger().onTrue(Commands.runOnce(flipperSubsystem::toggleInOutState));
        operatorController.rightTrigger().onTrue(Commands.runOnce(flipperSubsystem::toggleUpDownState));

        gridEntry.set(0);
        // Decrement the grid entry
        operatorController
                .leftBumper()
                .onTrue(RaiderCommands.runOnceAllowDisable(
                        () -> gridEntry.set(RaiderMathUtils.longClamp(gridEntry.get() - 1, 0, 8))));

        // Increment the grid entry
        operatorController
                .rightBumper()
                .onTrue(RaiderCommands.runOnceAllowDisable(
                        () -> gridEntry.set(RaiderMathUtils.longClamp(gridEntry.get() + 1, 0, 8))));

        // Cancel incoming as this is the highest priority
        operatorController
                .square()
                .toggleOnTrue(LEDCommandFactory.alternateColorCommand(0.5, Color.kPurple, Color.kBlack, ledSubsystem));
        operatorController
                .triangle()
                .toggleOnTrue(LEDCommandFactory.alternateColorCommand(0.5, Color.kGold, Color.kBlack, ledSubsystem));
    }

    private void configureDriving() {
        TunableDouble maxTranslationSpeedPercent = new TunableDouble("/speed/maxTranslation", 0.9, true);
        TunableDouble maxMaxAngularSpeedPercent = new TunableDouble("/speed/maxAngular", 0.3, true);

        BooleanSupplier armHigh = () -> liftSubsystem.getArmAngle().getRadians() > Units.degreesToRadians(-30.0);

        DoubleSupplier maxTranslationalSpeedSuppler = () -> maxTranslationSpeedPercent.get()
                * DriveTrainConstants.MAX_VELOCITY_METERS_SECOND
                * (driverController.leftBumper().getAsBoolean() ? 0.5 : 1);
        DoubleSupplier maxAngularSpeedSupplier = () -> maxMaxAngularSpeedPercent.get()
                * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND
                * (armHigh.getAsBoolean() ? 0.7 : 1.0);

        SupplierSlewRateLimiter rotationLimiter = new SupplierSlewRateLimiter(() -> {
            if (armHigh.getAsBoolean()) {
                return TeleopConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED * 0.4;
            } else {
                return TeleopConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED;
            }
        });
        VectorRateLimiter vectorRateLimiter = new VectorRateLimiter(() -> {
            if (armHigh.getAsBoolean()) {
                return TeleopConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED * 0.6;
            } else {
                return TeleopConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED;
            }
        });
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
                                .x()
                                .or(driverController.a())
                                .or(driverController.b())
                                .or(driverController.y()),
                        () -> {
                            if (driverController.y().getAsBoolean()) return Math.PI / 2;
                            if (driverController.b().getAsBoolean()) return Math.PI;
                            if (driverController.a().getAsBoolean()) return -Math.PI / 2;
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
                .onTrue(RaiderCommands.runOnceAllowDisable(() -> evaluateDriveStyle(driveCommandChooser.getSelected()))
                        .withName("Drive Style Checker"));
    }

    private Command rumbleDriverControllerCommand() {
        // return Commands.runEnd(
        //                 () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        //                 () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        //         .withTimeout(0.5)
        //         .ignoringDisable(true);
        return Commands.none();
    }

    private Command rumbleOperatorControllerCommand() {
        return Commands.runEnd(
                        () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
                        () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(0.5)
                .ignoringDisable(true);
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

    /**
     * Called from Robot.java when the alliance is detected to have changed.
     * @param newAlliance the new alliance
     */
    public void onAllianceChange(Alliance newAlliance) {}
}
