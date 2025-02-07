package frc.robot.oi;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.prime.dashboard.PrimeSendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

/**
 * A dashboard widget for building autonomous routines from a base of Pathplanner paths and named commands.
 */
public class PrimeAutoRoutine implements Sendable {

    private List<String> _routineSteps = new ArrayList<>();
    private String[] _pathNames = new String[0];
    private Map<String, Command> _namedCommands;

    private PrimeSendableChooser<String> _nextStepChooser;

    private boolean _addStepIsPressed;
    private BooleanEvent _addStepEvent;

    private boolean _removeLastStepIsPressed;
    private BooleanEvent _removeLastStepEvent;

    private boolean _clearRoutineIsPressed;
    private BooleanEvent _clearRoutineEvent;

    public PrimeAutoRoutine(Map<String, Command> commands) {
        _namedCommands = commands;

        List<String> paths = new ArrayList<>();
        var pathsPath = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        if (!pathsPath.exists() || !pathsPath.isDirectory()) {
            DriverStation.reportError("[AUTOBUILDER] No paths found for PrimeAutoRoutine!", false);

            return;
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

        _pathNames = paths.toArray(new String[0]);
    }

    /**
     * Sets the chooser for selecting the next step in the routine.
     */
    public void setChooser(PrimeSendableChooser<String> chooser) {
        _nextStepChooser = chooser;
        updateChooserOptions();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PrimeAutoRoutine");
        builder.addStringArrayProperty("Routine Steps", () -> _routineSteps.toArray(new String[0]), null);

        builder.addBooleanProperty("Add Step", () -> _addStepIsPressed, (value) -> _addStepIsPressed = value);
        _addStepEvent = new BooleanEvent(Robot.EventLoop, () -> _addStepIsPressed)
                .debounce(0.1);
        _addStepEvent.ifHigh(this::addRoutineStep);

        builder.addBooleanProperty("Remove Last Step", () -> _removeLastStepIsPressed,
                (value) -> _removeLastStepIsPressed = value);
        _removeLastStepEvent = new BooleanEvent(Robot.EventLoop, () -> _removeLastStepIsPressed)
                .debounce(0.1);
        _removeLastStepEvent.ifHigh(this::removeLastStep);

        builder.addBooleanProperty("Clear Routine", () -> _clearRoutineIsPressed,
                (value) -> _clearRoutineIsPressed = value);
        _clearRoutineEvent = new BooleanEvent(Robot.EventLoop, () -> _clearRoutineIsPressed)
                .debounce(0.1);
        _clearRoutineEvent.ifHigh(this::clearRoutine);

        builder.update();
    }

    /**
     * Adds the selected step to the routine.
     */
    private void addRoutineStep() {
        var newStep = _nextStepChooser.getSelected();
        if (newStep == null) {
            System.out.println("No step selected");
            return;
        }

        if (_routineSteps.size() > 0 && _routineSteps.get(_routineSteps.size() - 1) == newStep) {
            System.out.println("Cannot add the same step twice in a row");
            return;
        }

        _routineSteps.add(newStep);
        _addStepIsPressed = false;
        updateChooserOptions();
    }

    /**
     * Removes the last step from the routine.
     */
    private void removeLastStep() {
        if (_routineSteps.isEmpty()) {
            System.out.println("No steps to remove");
            return;
        }

        _routineSteps.remove(_routineSteps.size() - 1);
        System.out.println("Removed last step");
        _removeLastStepIsPressed = false;
        updateChooserOptions();
    }

    /**
     * Clears the current routine.
     */
    private void clearRoutine() {
        if (_routineSteps.isEmpty()) {
            System.out.println("Routine is already empty");
            return;
        }

        _routineSteps.clear();
        System.out.println("Cleared routine");
        _clearRoutineIsPressed = false;
        updateChooserOptions();
    }

    /**
     * Updates the options in the chooser based on the current routine steps.
     * If the routine is empty, only paths that start with "S" are shown.
     * If the routine is not empty, only paths that start with the last destination of the routine and commands are shown.
     */
    private void updateChooserOptions() {
        _nextStepChooser.clearOptions();

        if (_routineSteps.isEmpty()) {
            Map<String, String> startingPaths = new java.util.HashMap<>();
            for (var path : _pathNames) {
                if (path.startsWith("S") && path.contains("-to-")) {
                    startingPaths.put(path, path);
                }
            }
            _nextStepChooser.addOptions(startingPaths);
        } else {
            var currentLocation = getRoutineLastDestination();
            var validNextSteps = getCombinedRoutineOptions().stream().filter(
                    step -> !step.contains("-to-") || (!step.startsWith("S") && step.startsWith(currentLocation)))
                    .collect(Collectors.toMap(step -> step, step -> step));
            _nextStepChooser.addOptions(validNextSteps);
        }
    }

    /**
     * Gets the last path location in the routine.
     */
    public String getRoutineLastDestination() {
        for (int i = _routineSteps.size() - 1; i >= 0; i--) {
            var currentStep = _routineSteps.get(i);
            if (!stepIsCommand(currentStep)) {
                return currentStep.split("-to-")[1];
            }
        }

        return "";
    }

    /**
     * Gets the combined list of paths and commands that can be added to the routine.
     */
    public List<String> getCombinedRoutineOptions() {
        var options = new ArrayList<String>();

        for (var path : _pathNames) {
            options.add(path);
        }

        for (var command : _namedCommands.keySet()) {
            options.add(command);
        }

        return options;
    }

    /*
     * Combines paths and commands into a single routine
     */
    public Command exportCombinedAutoRoutine() {
        if (_routineSteps.isEmpty()) {
            DriverStation.reportError("[AUTOBUILDER] No steps in routine", false);

            return Commands.none();
        }

        var autoCommand = Commands.none();

        if (!AutoBuilder.isConfigured()) {
            DriverStation.reportError("[AUTOBUILDER] AutoBuilder is not configured", false);

            return Commands.none();
        }

        for (var step : _routineSteps) {
            if (stepIsPath(step)) {
                try {
                    // Load path from file
                    var path = PathPlannerPath.fromPathFile(step);
                    if (path == null) {
                        throw new Exception("Path not found");
                    }

                    // Flip path if on red alliance
                    if (Robot.onRedAlliance()) {
                        path = path.flipPath();
                    }

                    var followPathCommand = AutoBuilder.followPath(path);
                    if (followPathCommand == null) {
                        throw new Exception("Failed to build path command");
                    }

                    autoCommand = autoCommand.andThen(followPathCommand);
                } catch (Exception e) {
                    DriverStation.reportError("[AUTOBUILDER] Failed to load path: " + step, false);

                    return null;
                }
            } else if (stepIsCommand(step)) {
                // Step is a command
                var command = _namedCommands.get(step);
                if (command != null) {
                    autoCommand = autoCommand.andThen(command);
                } else {
                    DriverStation.reportError("[AUTOBUILDER] Command not found: " + step, false);

                    return null;
                }
            } else {
                DriverStation.reportError("[AUTOBUILDER] Step " + step + "not found in paths or commands.", false);

                return null;
            }
        }

        return autoCommand;
    }

    private boolean stepIsPath(String step) {
        return step.contains("-to-");
    }

    private boolean stepIsCommand(String step) {
        return _namedCommands.containsKey(step);
    }
}
