package frc.robot.oi;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PrimeAutoRoutine implements Sendable {

    private String[] _routineSteps = new String[0];
    private String[] _pathNames = new String[0];
    private Map<String, Command> _namedCommands;

    public PrimeAutoRoutine(Map<String, Command> commands) {
        _namedCommands = commands;

        List<String> paths = new ArrayList<>();
        var pathsPath = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        if (!pathsPath.exists() || !pathsPath.isDirectory()) {
            DriverStation.reportError("[AUTOBUILDER] No paths found for PrimeAutoRoutine!", false);

            return;
        }

        for (var file : pathsPath.listFiles()) {
            if (file.isFile() && file.getName().endsWith(".path")) {
                paths.add(file.getName().replace(".path", ""));
            }
        }

        _pathNames = paths.toArray(new String[0]);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PrimeAutoRoutine");
        builder.addStringArrayProperty("routineSteps", () -> _routineSteps, this::setRoutineSteps);
        builder.addStringArrayProperty("availablePaths", () -> _pathNames, null);
        builder.addStringArrayProperty("availableCommands", () -> _namedCommands.keySet().toArray(new String[0]), null);
    }

    public void setRoutineSteps(String[] steps) {
        _routineSteps = steps;
    }

    /*
     * Combines paths and commands into a single routine
     */
    public Command getCombinedAutoRoutine() {
        var autoCommand = Commands.none();

        for (var step : _routineSteps) {
            if (step.contains("-to-")) {
                // Step is a path
                try {
                    var path = PathPlannerPath.fromPathFile(step);
                    if (path == null)
                        throw new Exception("Path not found");

                    var followPathCommand = AutoBuilder.followPath(path);
                    if (followPathCommand == null)
                        throw new Exception("Failed to build path command");

                    autoCommand = autoCommand.andThen(followPathCommand);
                } catch (Exception e) {
                    DriverStation.reportError("[AUTOBUILDER] Failed to load path: " + step, false);

                    return null;
                }
            } else {
                // Step is a command
                var command = _namedCommands.get(step);
                if (command != null) {
                    autoCommand = autoCommand.andThen(command);
                } else {
                    DriverStation.reportError("[AUTOBUILDER] Command not found: " + step, false);

                    return null;
                }
            }
        }

        return autoCommand;
    }
}
