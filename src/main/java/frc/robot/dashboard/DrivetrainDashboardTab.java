package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.Robot;

public class DrivetrainDashboardTab extends DashboardTabBase {
    private GenericEntry _autoAlignEnabledEntry;
    private BooleanEvent _autoAlignEnabledEvent = null;
    private GenericEntry _autoAlignTargetEntry;

    public DrivetrainDashboardTab() {
        super("Drivetrain");

        _autoAlignEnabledEntry = createBooleanBox("AutoAlign Enabled", 0, 0, 2, 2);
        _autoAlignTargetEntry = createGyro("AutoAlign Angle", 2, 0, 4, 5);
    }

    public void setAutoAlignEnabled(boolean enabled) {
        _autoAlignEnabledEntry.setBoolean(enabled);
    }

    public boolean getAutoAlignEnabled() {
        return _autoAlignEnabledEntry.getBoolean(false);
    }

    public BooleanEvent getAutoAlignEnabledEvent() {
        if (_autoAlignEnabledEvent == null) {
            _autoAlignEnabledEvent = new BooleanEvent(Robot.EventLoop, getAutoAlignEnabledEvent())
                    .debounce(0.2);
        }

        return _autoAlignEnabledEvent;
    }

    public void setAutoAlignTarget(Rotation2d target) {
        _autoAlignTargetEntry.setDouble(target.getRadians());
    }
}
