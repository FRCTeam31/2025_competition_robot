package frc.robot.oi.routine;

import java.util.List;

/**
 * Different types of actions that can occur, used in the construction of a {@code RecordableUndoEntry}.
 */
public enum RecordableAction {
    kAdd,
    kRemove,
    kClear,
    kLoad;

    /**
     * Undoes an action by passing in the routine in the form of a list and the {@code RecordableUndoEntry}
     * for the change you want to undo. Data from the {@code RecordableUndoEntry} is used to alter the routine
     * to match its state before the change.
     * @param routine
     * @param undoEntry
     * @return
     */
    public List<String> performInverse(List<String> routine, RecordableUndoEntry undoEntry) {
        switch (this) {
            case kAdd:
                routine.remove(routine.size() - 1);
                break;
            case kRemove:
                routine.addAll(undoEntry.getUnpackedStep());
                break;
            case kClear:
                routine.addAll(undoEntry.getUnpackedStep());
                break;
            case kLoad:
                routine.clear();
                routine.addAll(undoEntry.getUnpackedStep());
                break;
        }
        return routine;
    }

    /**
     * Currently unimplemented. Could be used as a "redo" to preform an action based on the
     * {@code RecordableUndoEntry}.
     * @param routine
     * @param undoEntry
     * @return
     */
    public List<String> performAction(List<String> routine, RecordableUndoEntry undoEntry) {
        switch (this) {
            case kAdd:
                routine.addAll(undoEntry.getUnpackedStep());
                break;
            case kRemove:
                routine.remove(routine.size() - 1);
                break;
            case kClear:
                routine.clear();
                break;
            case kLoad:
                routine.clear();
                routine.addAll(undoEntry.getStep().getLoadedStep());
                break;
        }

        return routine;
    }
}
