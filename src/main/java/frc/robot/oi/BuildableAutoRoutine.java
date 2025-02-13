package frc.robot.oi;

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

    // History management
    private AutoRoutineHistory _routineHistory;
    private SendableChooser<String> _historyChooser;
    private SendableButton _applyHistoryButton;

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

                    // Set starting pose of the robot if path starts with "S"
                    if (step.startsWith("S")) {
                        // This resets the pose estimation to the first point of the starting path, instead of
                        // letting it try to reach the ending position from where it *thinks* that it started.
                        // Replicates the PP Auto "Reset Odometry" flag
                        var startingPose = path.getPathPoses().get(0);
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

        Container.TeleopDashboardSection.clearFieldPath();
        String routineName = DriverStation.isFMSAttached()
                ? DriverStation.getEventName() + " " + DriverStation.getMatchNumber()
                : "TestAuto" + System.currentTimeMillis();
        _routineHistory.addRoutineToHistory(routineName, _routineSteps);
        _historyChooser.addOption(routineName + " - " + String.join(", ", _routineSteps), routineName);
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
            _routineSteps.remove(_routineSteps.size() - 1);
            Container.TeleopDashboardSection.clearFieldPath();
            Elastic.sendInfo("Auto Routine", "Removed last routine step");
            updateChooserOptions();
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
            _routineSteps.clear();
            Container.TeleopDashboardSection.clearFieldPath();
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

        if (!_routineSteps.isEmpty()) {
            _routineSteps.clear();
        }

        Container.TeleopDashboardSection.clearFieldPath();
        for (var step : prevRoutineSteps) {
            _routineSteps.add(step);
        }

        Elastic.sendInfo("Auto Routine", "Applied historical routine: " + selectedRoutineName);

        updateChooserOptions();
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
            var pathfindCommand = Container.Swerve.pathfindToPoseCommand(startingPose);
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
            if (_routineSteps.isEmpty()) {
                // If the routine is empty, only show starting paths as options
                for (var path : _pathNames) {
                    if (stepIsStartingPath(path)) {
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

    private boolean stepIsPath(String step) {
        return step.contains("-to-");
    }

    private boolean stepIsCommand(String step) {
        return _namedCommands.containsKey(step);
    }

    private boolean stepIsStartingPath(String step) {
        return stepIsPath(step) && step.startsWith("S") && !step.startsWith("SRC");
    }

    private void onChooserChangeEvent(String newValue) {
        System.out.println("Chooser selected value changed: " + newValue);
        try {
            if (stepIsPath(newValue)) {
                var path = PathPlannerPath.fromPathFile(newValue);
                if (path == null) {
                    Elastic.sendError("Auto Routine", "Failed to load path: " + newValue);
                    return;
                }

                if (AutoBuilder.shouldFlip()) {
                    path = path.flipPath();
                }

                var finalPose = path.getPathPoses().get(path.getPathPoses().size() - 1);
                Container.TeleopDashboardSection.setFieldPath(path.getPathPoses());
                Container.TeleopDashboardSection.setFieldTargetPose(finalPose);
            }
        } catch (Exception e) {
            Elastic.sendWarning("Auto Routine", "Failed to display path for \"" + newValue + "\"");
            DriverStation.reportError("Failed to display path for \"" + newValue + "\"", e.getStackTrace());
        }
    }
}
