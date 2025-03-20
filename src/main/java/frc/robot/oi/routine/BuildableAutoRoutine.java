package frc.robot.oi.routine;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.prime.dashboard.SendableButton;
import org.prime.dashboard.ManagedSendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Container;
import frc.robot.Elastic;

/**
 * A dashboard widget for building autonomous routines from a base of Pathplanner paths and named commands.
 */
public class BuildableAutoRoutine {
    public static class VMap {
        public static Pose2d S1WaypointBlue = new Pose2d(7.575, 7.250, Rotation2d.fromDegrees(0));
        public static Pose2d S1RWaypointBlue = new Pose2d(7.575, 7.250, Rotation2d.fromDegrees(180));
        public static Pose2d S2WaypointBlue = new Pose2d(7.575, 6.150, Rotation2d.fromDegrees(0));
        public static Pose2d S2RWaypointBlue = new Pose2d(7.575, 6.150, Rotation2d.fromDegrees(180));
    }

    // Internal state
    private List<String> _routineSteps = new ArrayList<>();
    private String[] _pathNames = new String[0];
    private Map<String, Command> _namedCommands;
    private boolean _filterEnabled = true;
    private StartingLocationFilter _filteredStart = StartingLocationFilter.kNone;
    private RoutinePreviewMode _currentPreviewMode = RoutinePreviewMode.kFull;
    private List<RecordableUndoEntry> _undoRecord = new ArrayList<>();

    // Dashboard widgets
    private ManagedSendableChooser<String> _nextStepChooser;
    private SendableButton _addStepButton;
    private SendableButton _removeLastStepButton;
    private SendableButton _clearRoutineButton;
    private SendableButton _runRoutineManuallyButton;
    private SendableButton _returnToS1Button;
    private SendableButton _returnToS1RButton;
    private SendableButton _returnToS2Button;
    private SendableButton _returnToS2RButton;
    private SendableButton _undoLastChangeButton;
    private SendableChooser<Boolean> _toggleFilterSwitch;
    private SendableChooser<RoutinePreviewMode> _toggleRoutinePreviewMode;
    private SendableChooser<StartingLocationFilter> _toggleStartingFilterLocation;
    private SendableChooser<StartingDirectionFilter> _toggleStartingFilterDirection;

    // Auto Raise Elevator
    private AutoRoutineAutoRaiseElevator _autoRaiseElevator = new AutoRoutineAutoRaiseElevator();

    // History management
    private AutoRoutineHistory _routineHistory;
    private SendableChooser<String> _historyChooser;
    private SendableButton _applyHistoryButton;
    private SendableButton _clearHistoryButton;

    // Preloader management
    private SendableButton _preloadRoutineLoadButton;
    private SendableButton _preloadRoutineButton;
    private AutoRoutinePreloader _preloadedRoutine;

    public BuildableAutoRoutine(Map<String, Command> commands) {
        _namedCommands = commands;
        _pathNames = discoverPaths();

        _nextStepChooser = new ManagedSendableChooser<String>();
        _nextStepChooser.onChange(this::onChooserChangeEvent);
        Container.AutoDashboardSection.putData("Routine/Next Step Options", _nextStepChooser);
        updateChooserOptions();

        _addStepButton = new SendableButton("Add Step", this::addRoutineStep);
        Container.AutoDashboardSection.putData("Routine/Add Step", _addStepButton);

        _removeLastStepButton = new SendableButton("Remove Last Step", this::removeLastStep);
        Container.AutoDashboardSection.putData("Routine/Remove Last Step", _removeLastStepButton);

        _clearRoutineButton = new SendableButton("Clear Routine", this::clearRoutine);
        Container.AutoDashboardSection.putData("Routine/Clear Routine", _clearRoutineButton);

        _runRoutineManuallyButton = new SendableButton("Run Manually (Teleop)", this::runRoutineManually);
        Container.AutoDashboardSection.putData("Routine/Run Manually", _runRoutineManuallyButton);

        _returnToS1Button = new SendableButton("Return to S1", () -> pathfindToPose(VMap.S1WaypointBlue));
        Container.AutoDashboardSection.putData("Routine/Return to S1", _returnToS1Button);
        _returnToS1RButton = new SendableButton("Return to S1R", () -> pathfindToPose(VMap.S1RWaypointBlue));
        Container.AutoDashboardSection.putData("Routine/Return to S1R", _returnToS1RButton);
        _returnToS2Button = new SendableButton("Return to S2", () -> pathfindToPose(VMap.S2WaypointBlue));
        Container.AutoDashboardSection.putData("Routine/Return to S2", _returnToS2Button);
        _returnToS2RButton = new SendableButton("Return to S2R", () -> pathfindToPose(VMap.S2RWaypointBlue));
        Container.AutoDashboardSection.putData("Routine/Return to S2R", _returnToS2RButton);

        Container.AutoDashboardSection.putStringArray("Routine/Steps", _routineSteps.toArray(new String[0]));

        _routineHistory = new AutoRoutineHistory();
        _historyChooser = _routineHistory.getSendableChooser();
        Container.AutoDashboardSection.putData("Routine/History Options", _historyChooser);
        _applyHistoryButton = new SendableButton("Apply History", this::applyHistory);
        Container.AutoDashboardSection.putData("Routine/Apply History", _applyHistoryButton);
        _clearHistoryButton = new SendableButton("Clear History", _routineHistory::clearHistory);
        Container.AutoDashboardSection.putData("Routine/Clear History", _clearHistoryButton);

        _toggleFilterSwitch = new SendableChooser<>();
        _toggleFilterSwitch.setDefaultOption("Enable", true);
        _toggleFilterSwitch.addOption("Disable", false);
        _toggleFilterSwitch.onChange(this::toggleFilter);
        Container.AutoDashboardSection.putData("Routine/Filter Status", _toggleFilterSwitch);

        _toggleRoutinePreviewMode = new SendableChooser<>();
        _toggleRoutinePreviewMode.setDefaultOption("Single", RoutinePreviewMode.kSingle);
        _toggleRoutinePreviewMode.addOption("Full", RoutinePreviewMode.kFull);
        _toggleRoutinePreviewMode.addOption("After Image", RoutinePreviewMode.kAfterImage);
        _toggleRoutinePreviewMode.onChange(this::togglePreviewMode);
        Container.AutoDashboardSection.putData("Routine/Routine Preview Mode", _toggleRoutinePreviewMode);

        _toggleStartingFilterLocation = new SendableChooser<>();
        _toggleStartingFilterLocation.setDefaultOption("None", StartingLocationFilter.kNone);
        _toggleStartingFilterLocation.addOption("S1", StartingLocationFilter.kS1);
        _toggleStartingFilterLocation.addOption("S3", StartingLocationFilter.kS3);
        _toggleStartingFilterLocation.addOption("S4", StartingLocationFilter.kS4);
        _toggleStartingFilterLocation.addOption("S6", StartingLocationFilter.kS6);
        _toggleStartingFilterLocation.onChange(this::toggleStartingFilterLocation);
        Container.AutoDashboardSection.putData("Routine/Starting Filter/Location", _toggleStartingFilterLocation);

        _undoLastChangeButton = new SendableButton("Undo", this::undoLastChange);
        Container.AutoDashboardSection.putData("Routine/Undo", _undoLastChangeButton);

        _preloadedRoutine = new AutoRoutinePreloader();
        _preloadRoutineButton = new SendableButton("Preload Routine", this::preloadRoutine);
        Container.AutoDashboardSection.putData("Routine/Preloader/Preload Routine", _preloadRoutineButton);

        _preloadRoutineLoadButton = new SendableButton("Load", this::loadPreloadedRoutine);
        Container.AutoDashboardSection.putData("Routine/Preloader/Load", _preloadRoutineLoadButton);
    }

    /**
     * Toggles the filter for the next step chooser.
     * @param newValue
     */
    private void toggleFilter(Boolean newValue) {
        _filterEnabled = _toggleFilterSwitch.getSelected();
        updateChooserOptions();

        if (_filterEnabled) {
            Elastic.sendInfo("Filter Status", "Filter enabled");
        } else {
            Elastic.sendWarning("Filter Status",
                    "Filter disabled, this may cause unexpected behavior if path endpoints are not connected!",
                    8);
        }
    }

    /**
     * Toggles the preview mode for the field view.
     * @param newValue
     */
    private void togglePreviewMode(RoutinePreviewMode newValue) {
        _currentPreviewMode = _toggleRoutinePreviewMode.getSelected();
        updateFieldView(_nextStepChooser.getUserSelected());

        switch (_currentPreviewMode) {
            case kSingle:
                Elastic.sendInfo("Preview Mode", "Previewing single path");
                break;
            case kFull:
                Elastic.sendInfo("Preview Mode", "Previewing full routine");
                break;
            case kAfterImage:
                Elastic.sendInfo("Preview Mode", "Previewing after image");
                break;

        }
    }

    /**
    * Filters for a single starting location
    * @param newValue
    */
    private void toggleStartingFilterLocation(StartingLocationFilter newValue) {
        _filteredStart = _toggleStartingFilterLocation.getSelected();
        updateChooserOptions();

        switch (_filteredStart) {
            case kNone:
                Elastic.sendInfo("Starting Filter", "No starting location filter");
                break;
            case kS1:
                Elastic.sendInfo("Starting Filter", "Filtering for S1 starting location");
                break;
            case kS3:
                Elastic.sendInfo("Starting Filter", "Filtering for S3 starting location");
                break;
            case kS4:
                Elastic.sendInfo("Starting Filter", "Filtering for S4 starting location");
                break;
            case kS6:
                Elastic.sendInfo("Starting Filter", "Filtering for S6 starting location");
                break;
        }
    }

    /**
     * Handles undoing a previous change
     */
    private void undoLastChange() {
        Elastic.sendInfo("Undo", "Removed the last change made");

        try {
            var lastUndoRecord = _undoRecord.get(_undoRecord.size() - 1);

            _routineSteps = lastUndoRecord.getAction().performInverse(_routineSteps, lastUndoRecord);

            updateFieldView(_nextStepChooser.getUserSelected());
            updateChooserOptions();

            _undoRecord.remove(lastUndoRecord);
        } catch (Exception e) {
            Elastic.sendWarning("Undo", "Nothing to undo");
        }
    }

    /**
     * Find all available paths from the deploy/pathplanner/paths directory
     */
    private String[] discoverPaths() {
        List<String> paths = new ArrayList<>();
        var pathsPath = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        if (!pathsPath.exists() || !pathsPath.isDirectory()) {
            DriverStation.reportError("[AUTOBUILDER] No paths found for PrimeAutoRoutine!", false);

            return new String[0];
        }

        for (var file : pathsPath.listFiles()) {
            if (!file.isFile() || !file.getName().endsWith(".path")) {
                continue;
            }

            // Only add paths that are in the format "#-to-#"
            var pathName = file.getName().replace(".path", "");
            if (stepIsPath(pathName)) {
                paths.add(file.getName().replace(".path", ""));
            }
        }

        return paths.toArray(new String[0]);
    }

    /*
     * Combines paths and commands into a single routine
     */
    public Command exportCombinedAutoRoutine() {
        // reserve a command for if the builder encounters an error
        var errorCommand = Commands.runOnce(() -> {
            DriverStation.reportError("[AUTOBUILDER] Builder encountered an error. Running dummy command instead.",
                    false);
        });

        if (_routineSteps.isEmpty()) {
            Elastic.sendError("Auto Routine", "No steps in routine");

            return errorCommand;
        }

        // Make sure AutoBuilder is configured
        if (!AutoBuilder.isConfigured()) {
            Elastic.sendError("Auto Routine", "AutoBuilder is not configured");

            return errorCommand;
        }

        // Start with a command that immediately completes, just to give us a starting point
        Command autoCommand = new InstantCommand();
        for (var step : _routineSteps) {
            if (stepIsPath(step)) {
                try {
                    // Load path from file
                    var path = PathPlannerPath.fromPathFile(step);
                    if (path == null) {
                        throw new Exception("Path not found");
                    }

                    // Set starting pose of the robot if path is a starting path
                    if (stepIsStartingPath(step)) {
                        // This resets the pose estimation to the first point of the starting path, instead of
                        // letting it try to reach the ending position from where it *thinks* that it started.
                        // Replicates the PP Auto "Reset Odometry" flag
                        var startingPose = path.getPathPoses().get(0).transformBy(
                                new Transform2d(0, 0, path.getIdealStartingState().rotation()));

                        autoCommand = autoCommand.andThen(AutoBuilder.resetOdom(startingPose));
                    }

                    // AutoBuilder.followPath uses the configured AutoBuilder settings during the command, including
                    // automatic path flipping, holonomic correction PID, and global constraints.
                    var followPathCommand = AutoBuilder.followPath(path);
                    if (followPathCommand == null) {
                        throw new Exception("Failed to build path command");
                    }

                    autoCommand = autoCommand.andThen(followPathCommand);
                } catch (Exception e) {
                    DriverStation.reportError("[AUTOBUILDER] Failed to load path: " + step, false);

                    return errorCommand;
                }
            } else if (stepIsCommand(step)) {
                // Step is a command
                var command = _namedCommands.get(step);
                if (command != null) {
                    autoCommand = autoCommand.andThen(command);
                } else {
                    DriverStation.reportError("[AUTOBUILDER] Command not found: " + step, false);

                    return errorCommand;
                }
            } else {
                DriverStation.reportError("[AUTOBUILDER] Step " + step + "not found in paths or commands.", false);

                return errorCommand;
            }
        }

        autoCommand = autoCommand.andThen(Commands.print("Auto routine completed"));
        Container.TeleopDashboardSection.clearFieldPath();
        String routineName = DriverStation.isFMSAttached()
                ? DriverStation.getEventName() + " " + DriverStation.getMatchNumber()
                : "TestAuto" + System.currentTimeMillis();
        _routineHistory.addRoutineToHistory(routineName, _routineSteps);
        _historyChooser.addOption(routineName + " - " + String.join(", ", _routineSteps), routineName);

        _autoRaiseElevator.constructAutoElevatorPositionsList(_routineSteps);
        return autoCommand;
    }

    /**
     * Adds the selected step to the routine.
     */
    private void addRoutineStep() {
        var newStep = _nextStepChooser.getSelected();
        if (newStep == null) {
            Elastic.sendWarning("Auto Routine", "No step selected");
            return;
        }

        if (_routineSteps.size() > 0 && _routineSteps.get(_routineSteps.size() - 1) == newStep) {
            Elastic.sendWarning("Auto Routine", "Cannot add the same step twice in a row");
            return;
        }

        _undoRecord.add(new RecordableUndoEntry(RecordableAction.kAdd, new RecordableStepEntry(newStep)));
        _routineSteps.add(newStep);
        updateChooserOptions();
    }

    /**
     * Removes the last step from the routine.
     */
    private void removeLastStep() {
        if (_routineSteps.isEmpty()) {
            Elastic.sendWarning("Auto Routine", "No steps to remove");
        } else {
            _undoRecord.add(new RecordableUndoEntry(RecordableAction.kRemove,
                    new RecordableStepEntry(_routineSteps.get(_routineSteps.size() - 1))));
            _routineSteps.remove(_routineSteps.size() - 1);
            Elastic.sendInfo("Auto Routine", "Removed last routine step");
            updateChooserOptions();

            if (_routineSteps.isEmpty()) {
                Container.TeleopDashboardSection.clearFieldPath();
                Container.TeleopDashboardSection.setFieldRobotPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
                Container.TeleopDashboardSection.setFieldTargetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            } else {
                updateFieldView(_nextStepChooser.getUserSelected());
            }
        }
    }

    /**
     * Clears the current routine.
     */
    private void clearRoutine() {
        if (_routineSteps.isEmpty()) {
            Elastic.sendWarning("Auto Routine", "Routine is already empty");
            return;
        } else {
            _undoRecord.add(new RecordableUndoEntry(RecordableAction.kClear, new RecordableStepEntry(_routineSteps)));
            _routineSteps.clear();
            Container.TeleopDashboardSection.clearFieldPath();
            Container.TeleopDashboardSection.setFieldRobotPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            Container.TeleopDashboardSection.setFieldTargetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            System.out.println("Cleared routine");
            Elastic.sendInfo("Auto Routine", "Cleared routine");
            updateChooserOptions();
        }
    }

    /**
     * Runs the current routine manually.
     */
    private void runRoutineManually() {
        if (!DriverStation.isTeleopEnabled()) {
            Elastic.sendWarning("Auto Routine", "Robot is not enabled in Teleop mode");
            return;
        }

        var autoCommand = exportCombinedAutoRoutine();
        autoCommand.schedule();
    }

    /**
     * Overwrites the current routine steps with the steps from the selected historical routine.
     */
    private void applyHistory() {
        var selectedRoutineName = _historyChooser.getSelected();
        if (selectedRoutineName == null) {
            Elastic.sendWarning("Auto Routine", "No routine selected");
            return;
        }

        var prevRoutineSteps = _routineHistory.getRoutine(selectedRoutineName);
        if (prevRoutineSteps == null || prevRoutineSteps.length == 0) {
            Elastic.sendError("Auto Routine", "Failed to get routine steps for " + selectedRoutineName);
            return;
        }

        _undoRecord.add(new RecordableUndoEntry(RecordableAction.kLoad,
                new RecordableStepEntry(_routineSteps, List.of(prevRoutineSteps))));

        if (!_routineSteps.isEmpty()) {
            _routineSteps.clear();
        }

        Container.TeleopDashboardSection.clearFieldPath();
        for (var step : prevRoutineSteps) {
            _routineSteps.add(step);
        }

        Elastic.sendInfo("Auto Routine", "Applied historical routine: " + selectedRoutineName);

        updateChooserOptions();
        updateFieldView(_nextStepChooser.getUserSelected());
    }

    /**
     * Handles preloading a routine (saving to preferences)
     */
    private void preloadRoutine() {
        if (_routineSteps.isEmpty()) {
            Elastic.sendWarning("Routine Preloader", "You cannot preload an empty routine");
        } else {
            _preloadedRoutine.saveToPreferences(_routineSteps);
            Elastic.sendInfo("Routine Preloader",
                    "Preloaded routine, you can load it later by clicking load (this persists through power-cycles)",
                    8);
        }
    }

    /**
     * Handles loading a preloaded routine from preferences
     */
    private void loadPreloadedRoutine() {
        if (!_preloadedRoutine.readSavedRoutine().isEmpty()) {
            _undoRecord.add(new RecordableUndoEntry(RecordableAction.kLoad,
                    new RecordableStepEntry(_routineSteps, _preloadedRoutine.readSavedRoutine())));

            _routineSteps.clear();
            _routineSteps.addAll(_preloadedRoutine.readSavedRoutine());

            Elastic.sendInfo("Routine Preloader", "Loaded a preloaded routine");

            updateChooserOptions();
            updateFieldView(_nextStepChooser.getUserSelected());
        } else {
            Elastic.sendInfo("Routine Preloader", "No routine preloaded");
        }
    }

    /**
     * Returns the robot to a starting position
     * @param startingPose
     */
    private void pathfindToPose(Pose2d startingPose) {
        if (!DriverStation.isTeleopEnabled()) {
            Elastic.sendWarning("Auto Routine", "Robot is not enabled in Teleop mode");
            return;
        }

        try {
            var pathfindCommand = Container.Swerve.pathfindToPoseCommand(() -> startingPose, true);
            pathfindCommand.schedule();
        } catch (Exception e) {
            Elastic.sendError("Auto Routine", "Failed to pathfind to starting pose: " + e.getMessage());
        }
    }

    /**
     * Updates the options in the chooser based on the current routine steps.
     * If the routine is empty, only starting paths are shown.
     * If the routine is not empty, only paths that start with the last destination of the routine and commands are shown.
     */
    private void updateChooserOptions() {
        Container.AutoDashboardSection.putStringArray("Routine/Steps", _routineSteps.toArray(new String[0]));

        try {
            Map<String, String> validNextSteps = new HashMap<String, String>();
            if (_filterEnabled) {
                if (_routineSteps.isEmpty()) {
                    // If the routine is empty, only show starting paths as options
                    for (var path : _pathNames) {
                        if (stepIsStartingPath(path)) {
                            path.startsWith(_filteredStart.asName());
                            validNextSteps.put(path, path);
                        }
                    }
                } else {
                    // If the routine is not empty, show paths that start with the last destination of the routine and commands as options
                    for (var command : _namedCommands.keySet()) {
                        validNextSteps.put(command, command);
                    }

                    var currentLocation = getRoutineLastDestination();
                    for (var path : _pathNames) {
                        if (!stepIsStartingPath(path) && path.startsWith(currentLocation)) {
                            validNextSteps.put(path, path);
                        }
                    }
                }
            } else {
                // Adds all paths and commands as options if filter is disabled
                for (var command : _namedCommands.keySet()) {
                    validNextSteps.put(command, command);
                }

                for (var path : _pathNames) {
                    validNextSteps.put(path, path);
                }
            }

            // If no valid next steps are found, throw an exception
            if (validNextSteps.isEmpty()) {
                throw new Exception("No valid next steps found");
            }

            // Update the chooser options
            _nextStepChooser.replaceAllOptions(validNextSteps);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets the last path location in the routine.
     */
    private String getRoutineLastDestination() {
        for (int i = _routineSteps.size() - 1; i >= 0; i--) {
            var currentStep = _routineSteps.get(i);
            if (!stepIsCommand(currentStep)) {
                return currentStep.split("-to-")[1];
            }
        }

        return "";
    }

    /**
     * Returns the name of the last path in the routine.
     * @return
     */
    private String getRoutineLastPath() {
        for (int i = _routineSteps.size() - 1; i >= 0; i--) {
            var currentStep = _routineSteps.get(i);
            if (stepIsPath(currentStep)) {
                return currentStep;
            }
        }

        return "";
    }

    /**
     * Returns the name of the last path in the routine, but overlooks a certain number of paths.
     * (Skips the number given by {@code overlook}, 1 is second to last, 2 is third from last,
     * etc.) This is left over from previous logic and is no longer needed, but may prove useful in the future.
     * @param overlook
     * @return
     */
    private String getRoutineLastPath(int overlook) {
        for (int i = _routineSteps.size() - 1; i >= 0; i--) {
            var currentStep = _routineSteps.get(i);
            if (stepIsPath(currentStep)) {
                if (overlook > 0) {
                    overlook--;
                    continue;
                }

                return currentStep;
            }
        }

        return "";
    }

    private boolean stepIsPath(String step) {
        return step.contains("-to-");
    }

    private boolean stepIsCommand(String step) {
        return _namedCommands.containsKey(step);
    }

    private boolean stepIsStartingPath(String step) {
        return step.matches("^S\\dR?-to-.+");
    }

    private void onChooserChangeEvent(String newValue) {
        System.out.println("Chooser selected value changed: " + newValue);
        updateFieldView(newValue);
    }

    /**
     * Updates the field view based on the current routine steps and the user's selection.
     * Takes in a string for the user's current selection (This will most likely be {@code _nextStepChooser.getUserSelected()}).
     * @param newValue
     */
    private void updateFieldView(String newValue) {
        List<PathPlannerPath> _combinedRoutinePaths = new ArrayList<>();
        List<PathPlannerPath> _trimmedRoutinePaths = new ArrayList<>();
        List<Pose2d> _finalCombinedRoutinePoses = new ArrayList<>();

        try {
            // Clears the lists to ensure that they are empty before adding new values, even though they should
            // be empty at this point.
            _combinedRoutinePaths.clear();
            _trimmedRoutinePaths.clear();

            // Loops through every step in the routine picking out only the paths, does a few checks,
            // then combines them all into a single list.
            for (var step : _routineSteps) {
                if (stepIsPath(step)) {
                    var stepPath = PathPlannerPath.fromPathFile(step);

                    if (stepPath == null) {
                        Elastic.sendError("Auto Routine", "Failed to load path: " + newValue);
                        return;
                    }

                    if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
                        stepPath = stepPath.flipPath();
                    }

                    try {
                        _combinedRoutinePaths.add(stepPath);
                    } catch (Exception e) {
                        Elastic.sendError("Auto Routine", "Failed to load poses for path: " + step);
                    }

                }
            }

            // Creates a null PathPlannerPath object that is updated based on different conditions.
            PathPlannerPath path = null;

            // Decides weather path should be loaded from the user's selection or the last path in the routine, and weather
            // it should be added to the end of the combined paths list.
            if (newValue != null && stepIsPath(newValue)) {
                path = PathPlannerPath.fromPathFile(newValue);
                _combinedRoutinePaths.add(path);
            } else {
                // In this case, the path is already included in the combined paths list so it does not need to be added.
                path = PathPlannerPath.fromPathFile(getRoutineLastPath());
            }

            if (path == null) {
                Elastic.sendError("Auto Routine", "Failed to load path: " + newValue);
                return;
            }

            if (AutoBuilder.shouldFlip()) {
                path = path.flipPath();
            }

            // Defines the starting and ending poses of the path (either the user's selection or the last path in the routine).
            var startingPose = path.getPathPoses().get(0)
                    .transformBy(new Transform2d(0, 0, path.getIdealStartingState().rotation()));
            var finalPose = path.getPathPoses().get(path.getPathPoses().size() - 1)
                    .transformBy(new Transform2d(0, 0, path.getGoalEndState().rotation()));

            // Does post-processing on the combined paths list based on the current preview mode.
            // Any new preview modes will be implemented here by creating a new case for that mode and
            // defining how the list of all paths (including the current selection) should be altered
            // to achieve the desired look / outcome (_trimmedRoutinePaths is what will be shown / the output).
            switch (_currentPreviewMode) {
                case kSingle:
                    _trimmedRoutinePaths.add(path);
                    break;
                case kFull:
                    _trimmedRoutinePaths.addAll(_combinedRoutinePaths);
                    break;
                case kAfterImage:
                    _trimmedRoutinePaths.addAll(_combinedRoutinePaths);
                    if (_trimmedRoutinePaths.size() > 2) {
                        _trimmedRoutinePaths.subList(0, _trimmedRoutinePaths.size() - 2).clear();
                    }
                    break;
            }

            // Takes all paths from the post-processed list and creates a list of all their poses.
            // Theres should be little to no post-processing past this point as it is harder to
            // manipulate the poses than the paths.
            for (var finalPath : _trimmedRoutinePaths) {
                _finalCombinedRoutinePoses.addAll(finalPath.getPathPoses());
            }

            // Updates the field view with the combined poses of the routine (after post-processing),
            // the starting pose of the robot, and the ending pose of the path.
            Container.TeleopDashboardSection.setFieldPath(_finalCombinedRoutinePoses);
            Container.TeleopDashboardSection.setFieldRobotPose(startingPose);
            Container.TeleopDashboardSection.setFieldTargetPose(finalPose);

        } catch (Exception e) {
            Elastic.sendWarning("Auto Routine", "Failed to display path for \"" + newValue + "\"");
            DriverStation.reportError("Failed to display path for \"" + newValue + "\"", e.getStackTrace());
        }
    }
}
