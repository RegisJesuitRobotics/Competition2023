package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.MiscRobotTelemetryAndAlerts;
import frc.robot.telemetry.OverrunAlertManager;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.wrappers.TelemetryPneumaticHub;
import frc.robot.telemetry.wrappers.TelemetryPowerDistribution;
import frc.robot.utils.SparkMaxFlashManager;
import frc.robot.utils.wpilib.TreeTimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TreeTimedRobot {
    public static void startWNode(String nodeName) {
        instance.watchdog.addNode(nodeName);
    }

    public static void endWNode() {
        instance.watchdog.endCurrentNode();
    }

    private static Robot instance;

    private final double startTime;

    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private TelemetryPowerDistribution telemetryPowerDistribution;
    private TelemetryPneumaticHub telemetryPneumaticHub;
    private MiscRobotTelemetryAndAlerts miscRobotTelemetryAndAlerts;
    private OverrunAlertManager overrunAlertManager;
    private Alliance lastAlliance;

    public Robot() {
        startTime = Timer.getFPGATimestamp();

        instance = this;
    }

    /**
     * This method is run when the robot is first started up and should be used for any initialization
     * code.
     */
    @Override
    public void robotInit() {
        DataLogManager.log("*****START*****");

        DriverStation.silenceJoystickConnectionWarning(true);
        DataLogManager.logNetworkTables(false);
        DataLogManager.start();

        DataLog dataLog = DataLogManager.getLog();
        // Log all photon traffic and other things we specifically want to log
        NetworkTableInstance.getDefault().startConnectionDataLog(dataLog, "NTConnection");
        NetworkTableInstance.getDefault().startEntryDataLog(DataLogManager.getLog(), "/photonvision/", "photonvision/");
        NetworkTableInstance.getDefault().startEntryDataLog(DataLogManager.getLog(), "/toLog/", "toLog/");

        DriverStation.startDataLog(dataLog);

        telemetryPowerDistribution =
                new TelemetryPowerDistribution(MiscConstants.POWER_MODULE_ID, MiscConstants.POWER_MODULE_TYPE);
        telemetryPneumaticHub = new TelemetryPneumaticHub();
        miscRobotTelemetryAndAlerts = new MiscRobotTelemetryAndAlerts();
        overrunAlertManager = new OverrunAlertManager();

        //noinspection resource
        Notifier otherLoggingThread = new Notifier(() -> {
            telemetryPowerDistribution.logValues();
            telemetryPneumaticHub.logValues();
            miscRobotTelemetryAndAlerts.logValues();
        });
        otherLoggingThread.setName("Other Logging");
        otherLoggingThread.startPeriodic(0.1);

        SparkMaxFlashManager.init();
        robotContainer = new RobotContainer();

        lastAlliance = DriverStation.getAlliance();

        DataLogManager.log("RobotInit took " + (Timer.getFPGATimestamp() - startTime) + " seconds");
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
     * integrated updating.
     */
    @Override
    public void robotPeriodic() {
        overrunAlertManager.update(super.didLastLoopOverrun);

        if (DriverStation.getAlliance() != lastAlliance) {
            lastAlliance = DriverStation.getAlliance();
            robotContainer.onAllianceChange(lastAlliance);
        }

        watchdog.addNode("commandScheduler");
        CommandScheduler.getInstance().run();
        watchdog.endCurrentNode();

        watchdog.addNode("sendableTelemetry");
        SendableTelemetryManager.getInstance().update();
        watchdog.endCurrentNode();
    }

    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        telemetryPneumaticHub.enableCompressorAnalog(75, 120);
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        telemetryPneumaticHub.enableCompressorAnalog(75, 120);
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        telemetryPneumaticHub.enableCompressorAnalog(114, 120);
        CommandScheduler.getInstance().cancelAll();
    }

    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
