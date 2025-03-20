package frc.robot.oi.routine;

/**
 * Different locations for filtering starting locations when building an autonomus routine.
 * kNone is no filtering.
 */
public enum StartingLocationFilter {
    kNone,
    kS1,
    kS2,
    kS3;

    /**
     * Returns the name of the starting location enum without the k. (S1 instead of kS1)
     * @return
     */
    public String asName() {
        return this.toString().replace("k", "");
    }
}
