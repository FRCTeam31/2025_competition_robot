package frc.robot.oi;

import com.google.gson.Gson;
import java.util.ArrayList;
import java.util.List;

public class AutoRoutinePreproduction {
    private Gson _gson = new Gson();
    private List<String> _routine = new ArrayList<>();

    public AutoRoutinePreproduction() {

    }

    private void writeRoutineToJSON() {
        String _routineString = _gson.toJson(_routine);
    }

    private List<String> readRoutineFromJSON() {
        return null;
    }
}
