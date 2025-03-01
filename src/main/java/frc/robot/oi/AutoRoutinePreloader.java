package frc.robot.oi;

import java.util.ArrayList;
import java.util.List;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Preferences;

public class AutoRoutinePreloader {
    private static final ObjectMapper objectMapper = new ObjectMapper();

    private final String PRELOADER_PREF_KEY = "autoroutine-preloader";
    private List<String> _preloaded = new ArrayList<>();
    private String _fullRoutine = "";

    /**
     * Handles reading the saved routine if it exists, or creating a new preference if it does not.
     */
    public AutoRoutinePreloader() {
        if (!readSavedRoutine().isEmpty()) {
            readSavedRoutine();
        } else {
            Preferences.initString(PRELOADER_PREF_KEY, "");
        }
    }

    /**
     * Takes in a list of strings, an autonomus routine, and saves it to the robot's preferences.
     * This can then be loaded later though Elastic.
     * @param routine
     */
    public void saveToPreferences(List<String> routine) {
        _fullRoutine = "";

        for (var step : routine) {
            if (_fullRoutine == "") {
                _fullRoutine = step;
            } else {
                _fullRoutine = _fullRoutine + "," + step;
            }
        }

        Preferences.setString(PRELOADER_PREF_KEY, _fullRoutine);
    }

    /**
     * Returns a list of strings, an autonomus routine, that is read from the robot's preferences.
     * @return
     */
    public List<String> readSavedRoutine() {
        var savedRoutine = Preferences.getString(PRELOADER_PREF_KEY, "");
        _preloaded.clear();
        _preloaded.addAll(List.of(savedRoutine.split(",")));

        return _preloaded;
    }
}
