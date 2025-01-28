package frc.robot.dashboard;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Base class for dashboard tabs
 */
public abstract class DashboardTabBase {
    protected ShuffleboardTab _tab;

    /**
     * Creates a new dashboard tab with the given name
     */
    public DashboardTabBase(String tabName) {
        _tab = Shuffleboard.getTab(tabName);
    }

    // Helper methods for widget creation
    protected GenericEntry createBooleanBox(String title, int x, int y, int width, int height, String trueColor,
            String falseColor) {
        return _tab.add(title, false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(x, y)
                .withSize(width, height)
                .withProperties(Map.of("Color when true", trueColor, "Color when false", falseColor))
                .getEntry();
    }

    protected GenericEntry createBooleanBox(String title, int x, int y, int width, int height) {
        return _tab.add(title, false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(x, y)
                .withSize(width, height)
                .getEntry();
    }

    protected GenericEntry createGyro(String title, int x, int y, int width, int height) {
        return _tab.add(title, 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(x, y)
                .withSize(width, height)
                .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0,
                        "Minor tick spacing", 15.0))
                .getEntry();
    }

    protected GenericEntry createTextView(String title, int x, int y, int width, int height) {
        return _tab.add(title, 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(x, y)
                .withSize(width, height)
                .getEntry();
    }

    protected GenericEntry createDial(String title, double min, double max, int x, int y, int width, int height) {
        return _tab.add(title, 0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(x, y)
                .withSize(width, height)
                .withProperties(Map.of("Min", min, "Max", max))
                .getEntry();
    }

    protected GenericEntry createToggleSwitch(String title, int x, int y, int width, int height) {
        return _tab.add(title, true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(x, y)
                .withSize(width, height)
                .getEntry();
    }
}
