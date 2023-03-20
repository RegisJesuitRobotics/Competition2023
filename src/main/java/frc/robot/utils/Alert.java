// From Team 6328 Mechanical Advantage

package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.util.*;
import java.util.function.Predicate;

/** Class for managing persistent alerts to be sent over NetworkTables. */
public class Alert {
    private static final Map<String, AlertsGroup> groups = new HashMap<>();

    public static AlertsGroup getDefaultGroup() {
        return groups.get("Alerts");
    }

    private final AlertType type;

    private boolean active = false;
    private double activeStartTime = 0.0;
    private String text;

    /**
     * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
     * the appropriate entries will be added to NetworkTables.
     *
     * @param text Text to be displayed when the alert is active.
     * @param type Alert level specifying urgency.
     */
    public Alert(String text, AlertType type) {
        this("Alerts", text, type);
    }

    /**
     * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
     * entries will be added to NetworkTables.
     *
     * @param group Group identifier, also used as NetworkTables title
     * @param text Text to be displayed when the alert is active.
     * @param type Alert level specifying urgency.
     */
    public Alert(String group, String text, AlertType type) {
        if (!groups.containsKey(group)) {
            groups.put(group, new AlertsGroup());
            Shuffleboard.getTab("AlertsRaw").add(group, groups.get(group));
        }

        this.text = text;
        this.type = type;
        groups.get(group).alerts.add(this);
    }

    /** Sets whether the alert should currently be displayed. */
    public void set(boolean active) {
        if (active && !this.active) {
            activeStartTime = Timer.getFPGATimestamp();
        }
        this.active = active;
    }

    /** Updates current alert text. */
    public void setText(String text) {
        this.text = text;
    }

    public static class AlertsGroup implements Sendable {
        public final List<Alert> alerts = new ArrayList<>();

        private AlertsGroup() {}

        public String[] getStrings(AlertType type) {
            Predicate<Alert> activeFilter = (Alert x) -> x.type == type && x.active;
            Comparator<Alert> timeSorter = (Alert a1, Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
            return alerts.stream()
                    .filter(activeFilter)
                    .sorted(timeSorter)
                    .map((Alert a) -> a.text)
                    .toArray(String[]::new);
        }

        public boolean hasAnyErrors() {
            return alerts.stream().anyMatch((Alert a) -> a.type == AlertType.ERROR && a.active);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Alerts");
            builder.addStringArrayProperty("errors", () -> getStrings(AlertType.ERROR), null);
            builder.addStringArrayProperty("warnings", () -> getStrings(AlertType.WARNING), null);
            builder.addStringArrayProperty("infos", () -> getStrings(AlertType.INFO), null);
        }
    }

    /** Represents an alert's level of urgency. */
    public enum AlertType {
        /**
         * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
         * for problems which will seriously affect the robot's functionality and thus require immediate
         * attention.
         */
        ERROR,

        /**
         * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
         * type for problems which could affect the robot's functionality but do not necessarily require
         * immediate attention.
         */
        WARNING,

        /**
         * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
         * for problems which are unlikely to affect the robot's functionality, or any other alerts
         * which do not fall under "ERROR" or "WARNING".
         */
        INFO
    }
}
