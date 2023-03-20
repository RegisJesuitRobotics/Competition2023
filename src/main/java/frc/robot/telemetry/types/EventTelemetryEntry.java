package frc.robot.telemetry.types;

/** A String telemetry entry with lazy logging and NT disabled */
public class EventTelemetryEntry extends StringTelemetryEntry {
    public EventTelemetryEntry(String path) {
        super(path, false, false);
    }
}
