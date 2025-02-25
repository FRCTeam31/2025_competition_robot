package org.prime.dashboard;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The {@link SendableChooser} class is a useful tool for presenting a selection of options to the
 * {@link SmartDashboard}.
 *
 * <p>For instance, you may wish to be able to select between multiple autonomous modes. You can do
 * this by putting every possible Command you want to run as an autonomous into a {@link
 * SendableChooser} and then put it into the {@link SmartDashboard} to have a list of options appear
 * on the laptop. Once autonomous starts, simply ask the {@link SendableChooser} what the selected
 * value is.
 *
 * @param <V> The type of the values to be stored
 */
public class ManagedSendableChooser<V> implements Sendable, AutoCloseable {
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

    /** A map linking strings to the objects they represent. */
    private final Map<String, V> m_map = new LinkedHashMap<>();

    private String m_defaultChoice = "";
    private String m_selected;
    private String m_previousVal;
    private final int m_instance;
    private static final AtomicInteger s_instances = new AtomicInteger();
    private Consumer<V> m_listener;
    private final ReentrantLock m_mutex = new ReentrantLock();
    private Boolean m_hasHadInteraction = false;

    /** Instantiates a {@link SendableChooser}. */
    @SuppressWarnings("this-escape")
    public ManagedSendableChooser() {
        m_instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChooser", m_instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard} on the desktop, the
     * object will appear as the given name.
     *
     * @param name the name of the option
     * @param object the option
     */
    public void addOption(String name, V object) {
        m_map.put(name, object);
    }

    /**
     * Adds the map of objects to the list of options. On the {@link SmartDashboard} on the desktop, the
     * object will appear as the given name.
     *
     * @param name the name of the option
     * @param object the option
     */
    public void addOptions(Map<String, V> map) {
        for (Map.Entry<String, V> entry : map.entrySet()) {
            addOption(entry.getKey(), entry.getValue());
        }
    }

    public void replaceAllOptions(Map<String, V> map) {
        clearOptions();
        addOptions(map);
        m_selected = m_map.keySet().toArray(new String[0])[0];
        m_defaultChoice = m_selected;
        m_previousVal = m_selected;
    }

    /**
     * Removes all the options from the list.
     */
    public void clearOptions() {
        m_mutex.lock();
        try {
            m_selected = null;
            m_defaultChoice = "";
            m_previousVal = "";
            m_hasHadInteraction = false;
            m_map.clear();
        } finally {
            m_mutex.unlock();
        }
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
        m_defaultChoice = name;
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
                return m_map.get(m_selected);
            } else {
                return m_map.get(m_defaultChoice);
            }
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Returns the selected option. If the user has not made a selection, regardless of what is currently shown as
     * selected, it will return {@code null} rather than the default value.
     *
     * @return the option selected
     */
    public V getUserSelected() {
        m_mutex.lock();
        try {
            if (m_selected != null && m_hasHadInteraction) {
                return m_map.get(m_selected);
            } else if (!m_hasHadInteraction) {
                return null;
            } else {
                DriverStation.reportError(
                        "[SENDABLE CHOOSER] Tried to fetch the user's selection but it was null and there had previously been an interaction",
                        false);
                return null;
            }
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Returns the selected option's key. If there is none selected, it will return the default. If there is
     * none selected and no default, then it will return {@code null}.
     *
     * @return the option selected
     */
    public String getSelectedKey() {
        m_mutex.lock();
        try {
            return m_selected != null
                    ? m_selected
                    : m_defaultChoice;
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Bind a listener that's called when the selected value changes. Only one listener can be bound.
     * Calling this function will replace the previous listener.
     *
     * @param listener The function to call that accepts the new value
     */
    public void onChange(Consumer<V> listener) {
        m_mutex.lock();
        m_listener = listener;
        m_mutex.unlock();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.publishConstInteger(INSTANCE, m_instance);
        builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> m_map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(
                ACTIVE,
                this::getSelectedKey,
                null);
        builder.addStringProperty(
                SELECTED,
                this::getSelectedKey,
                val -> {
                    V choice;
                    Consumer<V> listener;
                    m_mutex.lock();
                    try {
                        m_selected = val;
                        if (!m_selected.equals(m_previousVal) && m_listener != null) {
                            choice = m_map.get(val);
                            listener = m_listener;
                            m_hasHadInteraction = true;
                        } else {
                            choice = null;
                            listener = null;
                        }
                        m_previousVal = val;
                    } finally {
                        m_mutex.unlock();
                    }
                    if (listener != null) {
                        listener.accept(choice);
                    }
                });
    }
}
