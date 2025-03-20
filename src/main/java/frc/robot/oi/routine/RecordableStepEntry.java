package frc.robot.oi.routine;

import java.util.ArrayList;
import java.util.List;

/**
 * Used to store data about an auto routine, for use as part of a {@code RecordableUndoEntry}.
 * This stores both a {@code parts} list that references the thing that was added, removed, or the entire routine before
 * it was cleared, and a {@code loadedParts} list that is used when loading to save multiple routine parts (before and after a load).
 */
public class RecordableStepEntry {
    private List<String> parts = new ArrayList<>();
    private List<String> loadedParts = new ArrayList<>();

    /**
     * Creates a {@code RecordableStepEntry} from a list of steps (paths or commands)
     * @param step
     */
    public RecordableStepEntry(List<String> step) {
        this.parts.clear();
        this.parts.addAll(step);
    }

    /**
     * Creates a {@code RecordableStepEntry} from single step (path or command)
     * @param step
     */
    public RecordableStepEntry(String step) {
        this.parts.clear();
        this.parts.add(step);
    }

    /**
     * Mainly used for a {@code RecordableUndoEntry} with an action of {@code kLoad}
     * Creates a {@code RecordableStepEntry} from two lists of steps, one for the routine before the load and one for
     * the routine that is being loaded.
     * @param stepPrior
     * @param stepLoaded
     */
    public RecordableStepEntry(List<String> stepPrior, List<String> stepLoaded) {
        this.parts.clear();
        this.loadedParts.clear();
        this.parts.addAll(stepPrior);
        this.loadedParts.addAll(stepLoaded);
    }

    /**
     * Gets the step(s) in the {@code RecordableStepEntry} as a list of Strings.
     * @return
     */
    public List<String> getStep() {
        return this.parts;
    }

    /**
     * Gets the loaded step(s) in the {@code RecordableStepEntry} as a list of Strings.
     * @return
     */
    public List<String> getLoadedStep() {
        return this.loadedParts;
    }
}
