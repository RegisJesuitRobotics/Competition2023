package frc.robot.telemetry.types.rich;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class ChassisSpeedsEntry {
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
}
