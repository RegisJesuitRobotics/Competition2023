package frc.robot.commands.drive.characterize;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.drive.characterize.modes.CharacterizationMode;
import frc.robot.commands.drive.characterize.modes.DynamicCharacterizationMode;
import frc.robot.commands.drive.characterize.modes.QuasistaticCharacterizationMode;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class DriveTrainSysIDCompatibleLoggerCommand extends CommandBase {
    private static final double METER_TO_ROTATION_FACTOR = 1.0 / DriveTrainConstants.WHEEL_DIAMETER_METERS * Math.PI;

    private final SwerveDriveSubsystem driveSubsystem;
    private final List<Double> dataBuffer = new ArrayList<>();

    private double acknowledgementNumber = 0;
    private CharacterizationMode currentCharacterizationMode;
    private double startTime;

    public DriveTrainSysIDCompatibleLoggerCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        dataBuffer.clear();

        acknowledgementNumber = SmartDashboard.getNumber("SysIdAckNumber", 0.0);
        SmartDashboard.putString("SysIdTelemetry", "");
        double voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        String type = SmartDashboard.getString("SysIdTestType", "Quasistatic");

        if (type.equals("Quasistatic")) {
            currentCharacterizationMode = new QuasistaticCharacterizationMode(voltageCommand);
        } else if (type.equals("Dynamic")) {
            currentCharacterizationMode = new DynamicCharacterizationMode(voltageCommand);
        } else {
            // Don't run anything
            currentCharacterizationMode = new DynamicCharacterizationMode(0.0);
        }

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp() - startTime;

        logValues(currentTime);
        double newVoltageRequest = currentCharacterizationMode.getVoltage(currentTime);
        driveSubsystem.setCharacterizationVoltage(newVoltageRequest);
    }

    public void logValues(double timestamp) {
        double[] voltages = driveSubsystem.getActualDriveVoltages();
        SwerveModulePosition[] positions = driveSubsystem.getModulePositions();
        SwerveModuleState[] states = driveSubsystem.getActualStates();

        // Order from:
        // https://github.com/wpilibsuite/sysid/blob/320b0e82f3b2a6dcad43e069898734f10891183f/sysid-library/src/main/cpp/logging/SysIdDrivetrainLogger.cpp#L27
        dataBuffer.add(timestamp);
        dataBuffer.add(voltages[0]);
        dataBuffer.add(voltages[1]);
        dataBuffer.add(positions[0].distanceMeters * METER_TO_ROTATION_FACTOR);
        dataBuffer.add(positions[1].distanceMeters * METER_TO_ROTATION_FACTOR);
        dataBuffer.add(states[0].speedMetersPerSecond * METER_TO_ROTATION_FACTOR);
        dataBuffer.add(states[1].speedMetersPerSecond * METER_TO_ROTATION_FACTOR);
        dataBuffer.add(driveSubsystem.getRawGyroAngle());
        dataBuffer.add(driveSubsystem.getRawGyroRate());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();

        String dataArrayString = dataBuffer.toString();
        String fullData = currentCharacterizationMode.getTestMetaData() + ";"
                + dataArrayString.substring(1, dataArrayString.length() - 1);
        SmartDashboard.putString("SysIdTelemetry", fullData);
        SmartDashboard.putNumber("SysIdAckNumber", ++acknowledgementNumber);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
