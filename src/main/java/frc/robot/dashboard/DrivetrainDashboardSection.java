package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.Robot;

public class DrivetrainDashboardSection extends DashboardSection {
    private final String _autoAlignKey = "AutoAlign Enabled";
    private BooleanEvent _autoAlignEnabledEvent = null;
    private final String _autoAlignTargetKey = "AutoAlign Angle";

    public DrivetrainDashboardSection() {
        super("Drivetrain");

        putBoolean(_autoAlignKey, false);
        putNumber(_autoAlignTargetKey, 0);
    }

    public void setAutoAlignEnabled(boolean enabled) {
        putBoolean(_autoAlignKey, enabled);
    }

    public boolean getAutoAlignEnabled() {
        return getBoolean(_autoAlignKey, false);
    }

    public BooleanEvent getAutoAlignEnabledEvent() {
        if (_autoAlignEnabledEvent == null) {
            _autoAlignEnabledEvent = new BooleanEvent(Robot.EventLoop, this::getAutoAlignEnabled)
                    .debounce(0.2);
        }

        return _autoAlignEnabledEvent;
    }

    public void setAutoAlignTarget(Rotation2d target) {
        putNumber(_autoAlignTargetKey, target.getRadians());
    }
}
