package frc.robot.oi;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Elastic;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.lang.reflect.Type;
import java.util.List;

public class AutoRoutinePreproduction {
    private Gson _gson = new Gson();

    private File _deployDir = Filesystem.getDeployDirectory();
    private File _preproductionPath = new File(_deployDir, "preproductionRoutine.json");

    public AutoRoutinePreproduction() {
        List<String> _routine = readRoutineFromJSON();

        if (_routine != null) {

        }
    }

    public void writeRoutineToJSON(List<String> _routine) {
        try {
            String _routineString = _gson.toJson(_routine);
            FileWriter _writer = new FileWriter(_preproductionPath);

            _writer.write(_routineString);
            _writer.close();
        } catch (Exception e) {
            Elastic.sendError("Routine Preproduction", "Unable to write to the preproduction file.");
        }
    }

    public List<String> readRoutineFromJSON() {
        try {
            FileReader _reader = new FileReader(_preproductionPath);
            Type _listType = new TypeToken<List<String>>() {
            }.getType();

            List<String> _readList = _gson.fromJson(_reader, _listType);

            _reader.close();
            return _readList;

        } catch (Exception e) {
            Elastic.sendError("Routine Preproduction", "Unable to read the preproduced file, it may not exist.");
            return null;
        }
    }
}
