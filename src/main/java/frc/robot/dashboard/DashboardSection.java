package frc.robot.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Base class for a SmartDashboard section that provides helper methods for creating values
 */
public class DashboardSection {
    protected String _tabName;

    /**
     * Creates a new dashboard tab with the given name
     */
    public DashboardSection(String tabName) {
        _tabName = tabName;
    }

    private String tabbedKey(String key) {
        return _tabName + "/" + key;
    }

    // Helper methods for widget creation
    public void putBoolean(String title, boolean defaultValue) {
        SmartDashboard.putBoolean(tabbedKey(title), defaultValue);
    }

    public boolean getBoolean(String title, boolean defaultValue) {
        return SmartDashboard.getBoolean(tabbedKey(title), defaultValue);
    }

    public void putNumber(String title, double defaultValue) {
        SmartDashboard.putNumber(tabbedKey(title), defaultValue);
    }

    public double getNumber(String title, double defaultValue) {
        return SmartDashboard.getNumber(tabbedKey(title), defaultValue);
    }

    public void putString(String title, String defaultValue) {
        SmartDashboard.putString(tabbedKey(title), defaultValue);
    }

    public void putStringArray(String title, String[] value) {
        SmartDashboard.putStringArray(tabbedKey(title), value);
    }

    public String getString(String title, String defaultValue) {
        return SmartDashboard.getString(tabbedKey(title), defaultValue);
    }

    public void putData(String title, Sendable data) {
        SmartDashboard.putData(tabbedKey(title), data);
    }
}
