package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import java.util.ArrayList;
import java.util.List;

/**
 * Add ability to log stuff like <code>Mechanism2d</code>
 */
public class SendableTelemetryManager {
    private static SendableTelemetryManager instance;

    private final List<SendableBuilderImpl> sendables = new ArrayList<>();

    public static SendableTelemetryManager getInstance() {
        if (instance == null) {
            instance = new SendableTelemetryManager();
        }
        return instance;
    }

    private SendableTelemetryManager() {}

    public void addSendable(String name, Sendable sendable) {
        SendableBuilderImpl sendableBuilder = new SendableBuilderImpl();
        sendableBuilder.setTable(NetworkTableInstance.getDefault().getTable("/toLog/" + name));

        sendable.initSendable(sendableBuilder);
        sendables.add(sendableBuilder);
    }

    public void update() {
        for (SendableBuilderImpl sendable : sendables) {
            sendable.update();
        }
    }
}
