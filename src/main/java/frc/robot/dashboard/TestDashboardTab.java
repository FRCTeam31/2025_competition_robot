package frc.robot.dashboard;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class TestDashboardTab extends DashboardTabBase {
    private final ShuffleboardLayout _autoLayout;

    public TestDashboardTab() {
        super("Test");
        _autoLayout = _tab.getLayout("Auto Routines", BuiltInLayouts.kList)
                .withSize(4, 16)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
    }

    public void setupTestAutos() {
        // Add all autos to the Test tab to be triggered manually
        var possibleAutos = AutoBuilder.getAllAutoNames();
        for (int i = 0; i < possibleAutos.size(); i++) {
            var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
            _autoLayout.add(possibleAutos.get(i), autoCommand);
        }
    }

    public ShuffleboardLayout addLayoutWithHiddenLabels(String name, int x, int y, int width, int height) {
        return _tab.getLayout(name, BuiltInLayouts.kList)
                .withPosition(x, y)
                .withSize(width, height)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
    }
}
