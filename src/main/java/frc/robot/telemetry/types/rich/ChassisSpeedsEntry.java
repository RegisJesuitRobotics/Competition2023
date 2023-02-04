package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.TelemetryEntry;

public class ChassisSpeedsEntry implements TelemetryEntry {
    private final DoubleTelemetryEntry xEntry;
    private final DoubleTelemetryEntry yEntry;
    private final DoubleTelemetryEntry omegaEntry;

    public ChassisSpeedsEntry(String path, boolean shouldNT) {
        path += "/";
        xEntry = new DoubleTelemetryEntry(path + "xSpeed", shouldNT);
        yEntry = new DoubleTelemetryEntry(path + "ySpeed", shouldNT);
        omegaEntry = new DoubleTelemetryEntry(path + "omegaSpeed", shouldNT);
    }

    public void append(ChassisSpeeds speeds) {
        xEntry.append(speeds.vxMetersPerSecond);
        yEntry.append(speeds.vyMetersPerSecond);
        omegaEntry.append(speeds.omegaRadiansPerSecond);
    }

    @Override
    public void close() {
        xEntry.close();
        yEntry.close();
        omegaEntry.close();
    }
}
