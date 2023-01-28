// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.wpilib;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.TreeTracer;
import java.io.Closeable;
import java.util.PriorityQueue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * A class that's a wrapper around a watchdog timer.
 *
 * <p>When the timer expires, a message is printed to the console and an optional user-provided
 * callback is invoked.
 *
 * <p>The watchdog is initialized disabled, so the user needs to call enable() before use.
 */
public class TreeWatchdog implements Closeable, Comparable<TreeWatchdog> {
    // Used for timeout print rate-limiting
    private static final long MIN_PRINT_PERIOD_MICRO_S = (long) 1e6;

    private double startTimeSeconds;
    private double timeoutSeconds;
    private double expirationTimeSeconds;
    private final Runnable callback;
    private double lastTimeoutPrintSeconds;

    boolean isExpired;

    boolean suppressTimeoutMessage;

    private final TreeTracer tracer;

    private static final PriorityQueue<TreeWatchdog> watchdogs = new PriorityQueue<>();
    private static final ReentrantLock queueMutex = new ReentrantLock();
    private static final int notifier;

    static {
        notifier = NotifierJNI.initializeNotifier();
        NotifierJNI.setNotifierName(notifier, "Watchdog");
        startDaemonThread(TreeWatchdog::schedulerFunc);
    }

    /**
     * Watchdog constructor.
     *
     * @param timeoutSeconds The watchdog's timeout in seconds with microsecond resolution.
     * @param callback This function is called when the timeout expires.
     */
    public TreeWatchdog(double timeoutSeconds, Runnable callback) {
        this.timeoutSeconds = timeoutSeconds;
        this.callback = callback;
        tracer = new TreeTracer();
    }

    @Override
    public void close() {
        disable();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof TreeWatchdog) {
            return Double.compare(expirationTimeSeconds, ((TreeWatchdog) obj).expirationTimeSeconds) == 0;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(expirationTimeSeconds);
    }

    @Override
    public int compareTo(TreeWatchdog rhs) {
        // Elements with sooner expiration times are sorted as lesser. The head of
        // Java's PriorityQueue is the least element.
        return Double.compare(expirationTimeSeconds, rhs.expirationTimeSeconds);
    }

    /**
     * Returns the time in seconds since the watchdog was last fed.
     *
     * @return The time in seconds since the watchdog was last fed.
     */
    public double getTime() {
        return Timer.getFPGATimestamp() - startTimeSeconds;
    }

    /**
     * Sets the watchdog's timeout.
     *
     * @param timeoutSeconds The watchdog's timeout in seconds with microsecond resolution.
     */
    public void setTimeout(double timeoutSeconds) {
        startTimeSeconds = Timer.getFPGATimestamp();
        tracer.resetEpochs();

        queueMutex.lock();
        try {
            this.timeoutSeconds = timeoutSeconds;
            isExpired = false;

            watchdogs.remove(this);
            expirationTimeSeconds = startTimeSeconds + this.timeoutSeconds;
            watchdogs.add(this);
            updateAlarm();
        } finally {
            queueMutex.unlock();
        }
    }

    /**
     * Returns the watchdog's timeout in seconds.
     *
     * @return The watchdog's timeout in seconds.
     */
    public double getTimeout() {
        queueMutex.lock();
        try {
            return timeoutSeconds;
        } finally {
            queueMutex.unlock();
        }
    }

    /**
     * Returns true if the watchdog timer has expired.
     *
     * @return True if the watchdog timer has expired.
     */
    public boolean isExpired() {
        queueMutex.lock();
        try {
            return isExpired;
        } finally {
            queueMutex.unlock();
        }
    }

    /**
     * Adds time since last epoch to the list printed by printEpochs().
     *
     * @see TreeTracer#addNode(String)
     * @param epochName The name to associate with the epoch.
     */
    public void addNode(String epochName) {
        tracer.addNode(epochName);
    }

    public void endCurrentNode() {
        tracer.endCurrentNode();
    }

    /**
     * Prints list of epochs added so far and their times.
     *
     * @see TreeTracer#printEpochs()
     */
    public void printEpochs() {
        tracer.printEpochs();
    }

    public void printEpochs(Consumer<String> output) {
        tracer.printEpochs(output);
    }

    /**
     * Resets the watchdog timer.
     *
     * <p>This also enables the timer if it was previously disabled.
     */
    public void reset() {
        enable();
    }

    /** Enables the watchdog timer. */
    public void enable() {
        startTimeSeconds = Timer.getFPGATimestamp();
        tracer.resetEpochs();

        queueMutex.lock();
        try {
            isExpired = false;

            watchdogs.remove(this);
            expirationTimeSeconds = startTimeSeconds + timeoutSeconds;
            watchdogs.add(this);
            updateAlarm();
        } finally {
            queueMutex.unlock();
        }
    }

    /** Disables the watchdog timer. */
    public void disable() {
        queueMutex.lock();
        try {
            watchdogs.remove(this);
            updateAlarm();
        } finally {
            queueMutex.unlock();
        }
    }

    /**
     * Enable or disable suppression of the generic timeout message.
     *
     * <p>This may be desirable if the user-provided callback already prints a more specific message.
     *
     * @param suppress Whether to suppress generic timeout message.
     */
    public void suppressTimeoutMessage(boolean suppress) {
        suppressTimeoutMessage = suppress;
    }

    private static void updateAlarm() {
        if (watchdogs.size() == 0) {
            NotifierJNI.cancelNotifierAlarm(notifier);
        } else {
            NotifierJNI.updateNotifierAlarm(notifier, (long) (watchdogs.peek().expirationTimeSeconds * 1e6));
        }
    }

    private static Thread startDaemonThread(Runnable target) {
        Thread inst = new Thread(target);
        inst.setDaemon(true);
        inst.start();
        return inst;
    }

    private static void schedulerFunc() {
        while (!Thread.currentThread().isInterrupted()) {
            long curTime = NotifierJNI.waitForNotifierAlarm(notifier);
            if (curTime == 0) {
                break;
            }

            queueMutex.lock();
            try {
                if (watchdogs.size() == 0) {
                    continue;
                }

                // If the condition variable timed out, that means a Watchdog timeout
                // has occurred, so call its timeout function.
                TreeWatchdog watchdog = watchdogs.poll();

                double now = curTime * 1e-6;
                if (now - watchdog.lastTimeoutPrintSeconds > MIN_PRINT_PERIOD_MICRO_S) {
                    watchdog.lastTimeoutPrintSeconds = now;
                    if (!watchdog.suppressTimeoutMessage) {
                        DriverStation.reportWarning(
                                String.format("Watchdog not fed within %.6fs\n", watchdog.timeoutSeconds), false);
                    }
                }

                // Set expiration flag before calling the callback so any
                // manipulation of the flag in the callback (e.g., calling
                // Disable()) isn't clobbered.
                watchdog.isExpired = true;

                queueMutex.unlock();
                watchdog.callback.run();
                queueMutex.lock();

                updateAlarm();
            } finally {
                queueMutex.unlock();
            }
        }
    }
}
