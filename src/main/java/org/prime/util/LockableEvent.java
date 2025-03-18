package org.prime.util;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Allows for the scheduling of an event that can be locked and unlocked.
 * After the event is unlocked the event will be passed to the runner.
 * 
 * @param <V> The type of event to be passed to the runner.
 */
public class LockableEvent<V> {
    private V _event;
    private Consumer<V> _runner;

    private boolean _locked;

    /**
     * Creates a new LockableEvent with the given event and runner.
     * 
     * @param event
     * @param runner
     */
    public LockableEvent(V event, Consumer<V> runner) {
        _event = event;
        _runner = runner;
    }

    /**
     * Creates a new LockableEvent with the given event, runner, and locked state.
     * 
     * @param event
     * @param runner
     * @param locked
     */
    public LockableEvent(V event, Consumer<V> runner, boolean locked) {
        _event = event;
        _runner = runner;
        _locked = locked;
    }

    /**
     * Sets the event to be passed to the runner.
     * 
     * @param event
     */
    public void setEvent(V event) {
        _event = event;
    }

    /**
     * Gets the event to be passed to the runner.
     * 
     * @return The event.
     */
    public V getEvent() {
        return _event;
    }

    /**
     * Locks the event, preventing any scheduled events from running until {@link #unlock()} is used.
     */
    public void lock() {
        _locked = true;
    }

    /**
     * Unlocks the event. Any previously scheduled events will now run.
     */
    public void unlock() {
        _locked = false;
    }

    /**
     * Returns true when the event is not locked.
     * @return BooleanSupplier
     */
    public BooleanSupplier isLocked() {
        return () -> _locked;
    }

    /**
     * Returns true when the event is locked.
     * @return BooleanSupplier
     */
    public BooleanSupplier isNotLocked() {
        return () -> !_locked;
    }

    /**
     * Returns true if the event is not null
     * @return BooleanSupplier
     */
    public BooleanSupplier isNotNull() {
        return () -> _event != null;
    }

    public void runEvent(V event) {
        _runner.accept(event);
    }

    /**
     * Schedules the event to be passed to the runner. Will run once the event is unlocked if currently locked, otherwise it will run instantly.
     */
    public Command scheduleLockableEventCommand() {
        return Commands.none().until(this.isNotLocked()).andThen(() -> this.runEvent(_event));

    }

}
