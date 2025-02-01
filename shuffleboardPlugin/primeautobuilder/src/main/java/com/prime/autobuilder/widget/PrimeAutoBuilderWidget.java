package com.prime.autobuilder.widget;

import edu.wpi.first.shuffleboard.api.widget.Description;
import edu.wpi.first.shuffleboard.api.widget.ParametrizedController;
import edu.wpi.first.shuffleboard.api.widget.SimpleAnnotatedWidget;
import java.util.Arrays;
import java.util.stream.Collectors;
import com.prime.autobuilder.data.PrimeAutoRoutineData;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.ListView;
import javafx.scene.layout.Pane;

@Description(name = "PrimeAutoBuilder", dataTypes = PrimeAutoRoutineData.class)
@ParametrizedController("PrimeAutoBuilder.fxml")
public final class PrimeAutoBuilderWidget extends SimpleAnnotatedWidget<PrimeAutoRoutineData> {
  @FXML
  private Pane root;
  @FXML
  private ComboBox<String> stepSelector;
  @FXML
  private ListView<String> stepsListView;
  @FXML
  private Button addStepButton;
  @FXML
  private Button removeStepButton;
  @FXML
  private Button clearButton;

  private final ObservableList<String> availableSteps = FXCollections.observableArrayList();

  @FXML
  private void initialize() {
    stepSelector.setItems(availableSteps);

    // Listen for data updates
    dataOrDefault.addListener((__, oldData, newData) -> {
      System.out.println("Data updated");
      System.out.println(newData.toHumanReadableString());
      updateAvailableChoices(newData);
    });

    stepsListView.itemsProperty().bind(dataOrDefault.map(r -> r.getRoutineStepsObservableList()));

    addStepButton.setOnAction(this::addStep);
    removeStepButton.setOnAction(this::removeStep);
    clearButton.setOnAction(this::clearRoutineSteps);

    // Initialize available steps
    updateAvailableChoices(dataOrDefault.get());
  }

  /**
   * Adds a step to the routine.
   * @param e The UI event that triggered this method
   */
  private void addStep(ActionEvent e) {
    String selectedStep = stepSelector.getValue();
    if (selectedStep != null) {
      var currentData = getData();
      setData(currentData.withRoutineStepAdded(selectedStep));
      updateAvailableChoices(currentData);
    } else {
      System.out.println("No step selected");
    }
  }

  /**
   * Removes the last step from the routine.
   * @param e The UI event that triggered this method
   */
  private void removeStep(ActionEvent e) {
    var currentData = getData();
    setData(currentData.withRemoveLastRoutineStep());
    updateAvailableChoices(currentData);
  }

  /**
   * Clears all steps from the routine.
   * @param e The UI event that triggered this method
   */
  private void clearRoutineSteps(ActionEvent e) {
    var currentData = getData();
    setData(currentData.withClearRoutineSteps());
    updateAvailableChoices(currentData);
  }

  /**
   * Updates the available steps based on the current routine.
   */
  private void updateAvailableChoices(PrimeAutoRoutineData newData) {
    availableSteps.clear();

    if (newData.getRoutineStepsList().isEmpty()) {
      // If the routine is empty, the only available steps are starting paths
      var startingPaths = newData.getStartingPaths();
      availableSteps.addAll(startingPaths);
    } else {
      // Otherwise, the available steps are those that start with the last location 
      // in the routine, are not starting paths, or are commands.
      var currentLocation = newData.getLastDestination();
      var validNextSteps = newData.getAllRoutineOptions().stream().filter(
          path -> isCommand(path) || (!path.startsWith("S") && path.startsWith(currentLocation)))
          .collect(Collectors.toList());
      availableSteps.addAll(validNextSteps);
    }

    System.out.println("Updated available steps: " + Arrays.toString(availableSteps.toArray()));
  }

  private boolean isCommand(String step) {
    return !step.contains("-to-");
  }

  @Override
  public Pane getView() {
    return root;
  }
}
