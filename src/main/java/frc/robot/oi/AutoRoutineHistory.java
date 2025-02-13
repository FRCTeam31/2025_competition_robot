package frc.robot.oi;

import java.io.File;
import java.nio.file.Files;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoRoutineHistory {
    private final String HISTORY_FILE_NAME = "autoroutine-history.txt";
    private LinkedHashMap<String, String[]> _history;

    public AutoRoutineHistory() {
        _history = readSavedHistory();
    }

    /**
     * Gets the history of auto routines in order of oldest to newest
     * @return
     */
    public Map<String, String[]> getHistory() {
        return _history;
    }

    /**
     * Gets the steps of a routine by name
     */
    public String[] getRoutine(String routineName) {
        return _history.get(routineName);
    }

    /**
     * Gets a sendable chooser with the history of auto routines
     * @return
     */
    public SendableChooser<String> getSendableChooser() {
        var chooser = new SendableChooser<String>();
        for (var entry : _history.entrySet()) {
            // The default name for the routine is the name of the routine followed by the routine steps
            var routineName = entry.getKey() + " - " + String.join(", ", entry.getValue());

            chooser.addOption(routineName, entry.getKey());
        }

        return chooser;
    }

    /**
     * Adds a routine to the end of the history
     * @param routineName
     * @param routineSteps
     */
    public void addRoutineToHistory(String routineName, List<String> routineSteps) {
        _history.put(routineName, routineSteps.toArray(new String[0]));
        saveHistoryToFile();
    }

    private File getHistoryFile() {
        var file = new File(".\\" + HISTORY_FILE_NAME);

        if (!file.exists()) {
            if (!file.canWrite()) {
                DriverStation.reportError("[AutoRoutineHistory] Cannot write new history file", false);

                return null;
            }

            try {
                file.createNewFile();
            } catch (Exception e) {
                DriverStation.reportError("[AutoRoutineHistory] Error creating history file", e.getStackTrace());

                return null;
            }
        }

        if (!file.isFile()) {
            DriverStation.reportError("[AutoRoutineHistory] History file is not a file", false);

            return null;
        }

        if (!file.canWrite()) {
            DriverStation.reportError("[AutoRoutineHistory] History file is not writable", false);

            return null;
        }

        if (!file.canRead()) {
            DriverStation.reportError("[AutoRoutineHistory] History file is not readable", false);

            return null;
        }

        return file;
    }

    /**
     * Reads the history of auto routines from the file in order of oldest to newest
     */
    private LinkedHashMap<String, String[]> readSavedHistory() {
        LinkedHashMap<String, String[]> history = new LinkedHashMap<>();
        var file = getHistoryFile();
        if (file != null) {
            try {
                // Read file
                var lines = Files.readAllLines(file.toPath());
                for (var line : lines) {
                    var routineName = line.split("-")[0];
                    var lineWithoutName = line.replaceFirst(routineName + "-", "");
                    history.put(routineName, lineWithoutName.split(","));
                }

                return history;
            } catch (Exception e) {
                DriverStation.reportError("[AutoRoutineHistory] Error reading history file", false);
            }
        } else {
            DriverStation.reportError("[AutoRoutineHistory] Failed to read history. File not found", false);
        }

        return history;
    }

    /**
     * Saves the current history to the on-disk file in order of oldest to newest
     */
    private void saveHistoryToFile() {
        var file = getHistoryFile();
        if (file != null) {
            try {
                var writer = Files.newBufferedWriter(file.toPath());
                for (var entry : _history.entrySet()) {
                    var routineName = entry.getKey();
                    var routineSteps = entry.getValue();
                    var line = routineName + "-" + String.join(",", routineSteps);
                    writer.write(line);
                    writer.newLine();
                }
                writer.close();
            } catch (Exception e) {
                DriverStation.reportError("[AutoRoutineHistory] Error saving history file", false);
            }
        } else {
            DriverStation.reportError("[AutoRoutineHistory] Failed to save history file. File not found", false);
        }
    }
}
