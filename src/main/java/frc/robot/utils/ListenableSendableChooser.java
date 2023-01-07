// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
// This is temporary

/**
 * The {@link ListenableSendableChooser} class is a useful tool for presenting a selection of
 * options to the {@link SmartDashboard}.
 *
 * <p>For instance, you may wish to be able to select between multiple autonomous modes. You can do
 * this by putting every possible Command you want to run as an autonomous into a {@link
 * ListenableSendableChooser} and then put it into the {@link SmartDashboard} to have a list of
 * options appear on the laptop. Once autonomous starts, simply ask the {@link
 * ListenableSendableChooser} what the selected value is.
 *
 * @param <V> The type of the values to be stored
 */
public class ListenableSendableChooser<V> implements NTSendable, AutoCloseable {
    /** The key for the default value. */
    private static final String DEFAULT = "default";
    /** The key for the selected option. */
    private static final String SELECTED = "selected";
    /** The key for the active option. */
    private static final String ACTIVE = "active";
    /** The key for the option array. */
    private static final String OPTIONS = "options";
    /** The key for the instance number. */
    private static final String INSTANCE = ".instance";
    /** A map linking strings to the objects the represent. */
    private final Map<String, V> map = new LinkedHashMap<>();

    private String defaultChoice = "";
    private final int instance;
    private static final AtomicInteger s_instances = new AtomicInteger();

    private boolean hasNewValue = true;

    /** Instantiates a {@link ListenableSendableChooser}. */
    public ListenableSendableChooser() {
        instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChooser", instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
        m_mutex.lock();
        try {
            for (StringPublisher pub : m_activePubs) {
                pub.close();
            }
        } finally {
            m_mutex.unlock();
        }
    }

    public boolean hasNewValue() {
        boolean temp = hasNewValue;
        hasNewValue = false;
        return temp;
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard} on the desktop, the
     * object will appear as the given name.
     *
     * @param name the name of the option
     * @param object the option
     */
    public void addOption(String name, V object) {
        map.put(name, object);
    }

    /**
     * Adds the given object to the list of options and marks it as the default. Functionally, this is
     * very close to {@link #addOption(String, Object)} except that it will use this as the default
     * option if none other is explicitly selected.
     *
     * @param name the name of the option
     * @param object the option
     */
    public void setDefaultOption(String name, V object) {
        requireNonNullParam(name, "name", "setDefaultOption");

        defaultChoice = name;
        addOption(name, object);
    }

    /**
     * Returns the selected option. If there is none selected, it will return the default. If there is
     * none selected and no default, then it will return {@code null}.
     *
     * @return the option selected
     */
    public V getSelected() {
        m_mutex.lock();
        try {
            if (m_selected != null) {
                return map.get(m_selected);
            } else {
                return map.get(defaultChoice);
            }
        } finally {
            m_mutex.unlock();
        }
    }

    private String m_selected;
    private final List<StringPublisher> m_activePubs = new ArrayList<>();
    private final ReentrantLock m_mutex = new ReentrantLock();

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        IntegerPublisher instancePub = new IntegerTopic(builder.getTopic(INSTANCE)).publish();
        instancePub.set(instance);
        builder.addCloseable(instancePub);
        builder.addStringProperty(DEFAULT, () -> defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(
                ACTIVE,
                () -> {
                    m_mutex.lock();
                    try {
                        if (m_selected != null) {
                            return m_selected;
                        } else {
                            return defaultChoice;
                        }
                    } finally {
                        m_mutex.unlock();
                    }
                },
                null);
        m_mutex.lock();
        try {
            m_activePubs.add(new StringTopic(builder.getTopic(ACTIVE)).publish());
        } finally {
            m_mutex.unlock();
        }
        builder.addStringProperty(SELECTED, null, val -> {
            m_mutex.lock();
            try {
                m_selected = val;
                hasNewValue = true;
                for (StringPublisher pub : m_activePubs) {
                    pub.set(val);
                }
            } finally {
                m_mutex.unlock();
            }
        });
    }
}
