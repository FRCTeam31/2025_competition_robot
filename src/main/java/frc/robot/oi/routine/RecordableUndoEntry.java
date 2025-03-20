package frc.robot.oi.routine;

import java.util.List;

/**
 * Used to store data required for a undo entry.
 * There is an action, an enum that defines the action that was performed,
 * and a step, a {@code RecordableStepEntry} that stores data about the routine.
 * This data can be used to both undo an action (how it is currently used), or
 * reconstruct an action (redo, currently not implemented).
 */
public class RecordableUndoEntry {
    private RecordableAction action;
    private RecordableStepEntry step;

    public RecordableUndoEntry(RecordableAction action, RecordableStepEntry step) {
        this.action = action;
        this.step = step;
    }

    public RecordableAction getAction() {
        return this.action;
    }

    public RecordableStepEntry getStep() {
        return this.step;
    }

    public List<String> getUnpackedStep() {
        return this.step.getStep();
    }
}
