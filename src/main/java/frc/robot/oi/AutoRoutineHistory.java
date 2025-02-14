package frc.robot.oi;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoRoutineHistory {
    private static final ObjectMapper objectMapper = new ObjectMapper();

    private final String HISTORY_PREF_KEY = "autoroutine-history";
    private Map<String, String[]> _history;

    public AutoRoutineHistory() {
        readSavedHistory();
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
        saveHistory();
    }

    /**
     * Reads the history of auto routines from the file in order of oldest to newest
     */
    private void readSavedHistory() {
        var historyFromPreferences = Preferences.getString(HISTORY_PREF_KEY, null);
        if (historyFromPreferences == null) {
            _history = new HashMap<>();
            saveHistory();

            historyFromPreferences = Preferences.getString(HISTORY_PREF_KEY, null);
        }

        if (historyFromPreferences != null && !historyFromPreferences.isBlank()) {
            try {
                // Read string
                var parsedHistory = objectMapper.readValue(historyFromPreferences,
                        new TypeReference<Map<String, String[]>>() {
                        });

                if (parsedHistory == null) {
                    throw new Exception("Failed to read history object from preferences");
                }

                _history = parsedHistory;
            } catch (Exception e) {
                DriverStation.reportError("[AutoRoutineHistory] Error reading history", false);
            }
        } else {
            _history = new HashMap<>();
            DriverStation.reportWarning("[AutoRoutineHistory] Started new routine history", false);
        }
    }

    /**
     * Saves the current history to the on-disk file in order of oldest to newest
     */
    private void saveHistory() {
        if (_history != null) {
            try {
                var historyAsString = objectMapper.writeValueAsString(_history);
                Preferences.setString(HISTORY_PREF_KEY, historyAsString);
            } catch (Exception e) {
                DriverStation.reportError("[AutoRoutineHistory] Error saving history to Preferences", false);
            }
        } else {
            DriverStation.reportError("[AutoRoutineHistory] Failed to save history. Internal history is [null]", false);
        }
    }
}
