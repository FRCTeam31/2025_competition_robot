package frc.robot.oi;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Preferences;

public class AutoRoutinePreloader {
    private static final ObjectMapper objectMapper = new ObjectMapper();

    private final String PRELOADER_PREF_KEY = "autoroutine-preloader";
    private List<String> _preloaded = new ArrayList<>();
    private String _fullRoutine = "";

    public AutoRoutinePreloader() {
        if (!readSavedRoutine().isEmpty()) {
            readSavedRoutine();
        } else {
            Preferences.initString(PRELOADER_PREF_KEY, "");
        }
    }

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

    public List<String> readSavedRoutine() {
        var savedRoutine = Preferences.getString(PRELOADER_PREF_KEY, "");
        _preloaded.clear();
        _preloaded.addAll(List.of(savedRoutine.split(",")));

        return _preloaded;
    }
}
