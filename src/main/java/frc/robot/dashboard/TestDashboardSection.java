package frc.robot.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class TestDashboardSection extends DashboardSection {

    public TestDashboardSection() {
        super("Test");
        setupTestAutos();
    }

    public void setupTestAutos() {
        // Add all autos to the Test tab to be triggered manually
        var possibleAutos = AutoBuilder.getAllAutoNames();
        for (int i = 0; i < possibleAutos.size(); i++) {
            var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
            putData(autoCommand.getName(), autoCommand);
        }
    }
}
