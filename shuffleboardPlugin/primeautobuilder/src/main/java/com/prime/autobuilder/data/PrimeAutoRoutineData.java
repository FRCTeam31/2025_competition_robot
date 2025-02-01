package com.prime.autobuilder.data;

import edu.wpi.first.shuffleboard.api.data.ComplexData;
import edu.wpi.first.shuffleboard.api.util.Maps;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Represents a complex data type for a Prime Movers autonomous routine.
 */
public final class PrimeAutoRoutineData extends ComplexData<PrimeAutoRoutineData> {

  public static final String ROUTINE_STEPS_KEY = "routineSteps";
  public static final String AVAILABLE_PATHS_KEY = "availablePaths";
  public static final String AVAILABLE_COMMANDS_KEY = "availableCommands";

  private final List<String> routineSteps = new ArrayList<>();
  private final List<String> availablePaths = new ArrayList<>();
  private final List<String> availableCommands = new ArrayList<>();

  /**
   * Creates a new PrimeAutoRoutine with a routine, available paths, and available commands.
   * @param routineSteps The steps in the routine
   * @param availablePaths The available paths to choose from
   * @param availableCommands The available commands to choose from
   */
  public PrimeAutoRoutineData(String[] routineSteps, String[] availablePaths,
      String[] availableCommands) {
    if (routineSteps != null && routineSteps.length > 0) {
      Collections.addAll(this.routineSteps, routineSteps);
    }

    if (availablePaths != null && availablePaths.length > 0) {
      Collections.addAll(this.availablePaths, availablePaths);
    }

    if (availableCommands != null && availableCommands.length > 0) {
      Collections.addAll(this.availableCommands, availableCommands);
    }
  }

  /**
   * Creates a new PrimeAutoRoutine from a map.
   * @param map The map to create the routine from 
   */
  public PrimeAutoRoutineData(Map<String, Object> map) {
    this(Maps.getOrDefault(map, ROUTINE_STEPS_KEY, new String[0]),
        Maps.getOrDefault(map, AVAILABLE_PATHS_KEY, new String[0]),
        Maps.getOrDefault(map, AVAILABLE_COMMANDS_KEY, new String[0]));
  }

  /**
   * Gets the steps in this routine.
   */
  public List<String> getRoutineStepsList() {
    return routineSteps;
  }

  /**
   * Gets the steps in this routine.
   */
  public ObservableList<String> getRoutineStepsObservableList() {
    return FXCollections.observableList(routineSteps);
  }

  /**
   * Gets the available paths.
   */
  public List<String> getAvailablePaths() {
    return availablePaths;
  }

  /**
   * Gets the starting paths (paths that start with "S" and contain "-to-").
   */
  public List<String> getStartingPaths() {
    List<String> startingPaths = new ArrayList<>();
    for (var path : availablePaths) {
      if (path.startsWith("S") && path.contains("-to-")) {
        startingPaths.add(path);
      }
    }

    return startingPaths;
  }

  /**
   * Gets the available commands.
   */
  public List<String> getAvailableCommands() {
    return availableCommands;
  }

  /**
   * Gets all available options for the routine.
   */
  public List<String> getAllRoutineOptions() {
    List<String> collection = FXCollections.observableArrayList();
    for (var path : availablePaths) {
      collection.add(path);
    }
    for (var command : availableCommands) {
      collection.add(command);
    }

    return collection;
  }

  /**
   * Gets the last path location in the routine.
   */
  public String getLastDestination() {
    for (int i = routineSteps.size() - 1; i >= 0; i--) {
      var currentStep = routineSteps.get(i);
      if (currentStep.contains("-to-")) {
        return currentStep.split("-to-")[1];
      } else {
        continue;
      }
    }

    return "";
  }

  /**
   * Creates a new PrimeAutoRoutine with the given steps
   * @param routineSteps The steps in the routine
   */
  public PrimeAutoRoutineData withRoutineSteps(String[] routineSteps) {
    return new PrimeAutoRoutineData(routineSteps, availablePaths.toArray(new String[0]),
        availableCommands.toArray(new String[0]));
  }

  /**
   * Creates a new PrimeAutoRoutine with a step added to the routine
   * @param step The step to add to the routine
   */
  public PrimeAutoRoutineData withRoutineStepAdded(String step) {
    System.out.println("Added step: " + step);
    routineSteps.add(step);

    return new PrimeAutoRoutineData(routineSteps.toArray(new String[0]),
        availablePaths.toArray(new String[0]), availableCommands.toArray(new String[0]));
  }

  /**
   * Creates a new PrimeAutoRoutine with the last step removed from the routine
   */
  public PrimeAutoRoutineData withRemoveLastRoutineStep() {
    if (!routineSteps.isEmpty()) {
      routineSteps.remove(routineSteps.size() - 1);
      System.out.println("Removed last step");

      return new PrimeAutoRoutineData(routineSteps.toArray(new String[0]),
          availablePaths.toArray(new String[0]), availableCommands.toArray(new String[0]));
    }

    // If there are no steps to remove, do nothing and return the current data
    return this;
  }

  /**
   * Creates a new PrimeAutoRoutine with all steps removed from the routine
   */
  public PrimeAutoRoutineData withClearRoutineSteps() {
    if (routineSteps.isEmpty()) {
      // If there are no steps to clear, do nothing and return the current data
      return this;
    }

    routineSteps.clear();
    System.out.println("Cleared routine steps");

    return new PrimeAutoRoutineData(routineSteps.toArray(new String[0]),
        availablePaths.toArray(new String[0]), availableCommands.toArray(new String[0]));
  }

  @Override
  public String toHumanReadableString() {
    return "Routine: " + String.join(", ", routineSteps);
  }

  @Override
  public Map<String, Object> asMap() {
    return Maps.<String, Object>builder()
        .put(ROUTINE_STEPS_KEY, routineSteps.toArray(new String[0]))
        .put(AVAILABLE_PATHS_KEY, availablePaths.toArray(new String[0]))
        .put(AVAILABLE_COMMANDS_KEY, availableCommands.toArray(new String[0])).build();
  }

  @Override
  public int hashCode() {
    int result = routineSteps.hashCode();
    result = 31 * result + availableCommands.hashCode();
    result = 31 * result + availablePaths.hashCode();
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null || getClass() != obj.getClass()) {
      return false;
    }

    final PrimeAutoRoutineData that = (PrimeAutoRoutineData) obj;

    return Arrays.equals(this.routineSteps.toArray(new String[0]),
        that.routineSteps.toArray(new String[0]))
        && Arrays.equals(this.availablePaths.toArray(new String[0]),
            that.availablePaths.toArray(new String[0]))
        && Arrays.equals(this.availableCommands.toArray(new String[0]),
            that.availableCommands.toArray(new String[0]));
  }
}
