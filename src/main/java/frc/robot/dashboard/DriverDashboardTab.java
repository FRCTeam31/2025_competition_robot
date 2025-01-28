package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import com.pathplanner.lib.auto.AutoBuilder;

public class DriverDashboardTab extends DashboardTabBase {
        private final Field2d _fieldWidget;
        private SendableChooser<Command> _autoChooser;

        // Widgets
        private GenericEntry _allianceBox;
        private GenericEntry _headingGyro;
        private GenericEntry _rearApTagIdField;
        private GenericEntry _frontApTagIdField;
        private GenericEntry _rearApTagOffsetDial;
        private GenericEntry _frontPoseEstimationSwitch;
        private BooleanEvent _frontPoseEstimationEvent = null;
        private GenericEntry _rearPoseEstimationSwitch;
        private BooleanEvent _rearPoseEstimationEvent = null;

        public DriverDashboardTab() {
                super("Driver");
                _fieldWidget = new Field2d();

                _allianceBox = createBooleanBox("Alliance", 15, 0, 2, 3, "#FF0000", "#0000FF");
                _headingGyro = createGyro("Current Heading", 12, 0, 3, 3);
                _rearApTagIdField = createTextView("Rear APTag", 12, 3, 2, 1);
                _frontApTagIdField = createTextView("Front APTag", 14, 3, 2, 1);
                _rearApTagOffsetDial = createDial("Rear APTag X Offset", -29.8, 29.8, 12, 4, 2, 3);
                _frontPoseEstimationSwitch = createToggleSwitch("F Pose Est.", 14, 4, 2, 1);
                _rearPoseEstimationSwitch = createToggleSwitch("R Pose Est.", 14, 5, 2, 1);

                // Add complex widgets like the field view
                _tab.add("Field", _fieldWidget)
                                .withWidget(BuiltInWidgets.kField)
                                .withPosition(0, 0)
                                .withSize(12, 6);
                _autoChooser = AutoBuilder.buildAutoChooser("Default");
                _tab.add(_autoChooser)
                                .withWidget(BuiltInWidgets.kComboBoxChooser)
                                .withPosition(0, 6)
                                .withSize(5, 2);
        }

        public Command getSelectedAuto() {
                return _autoChooser.getSelected();
        }

        public void setFieldRobotPose(Pose2d pose) {
                _fieldWidget.setRobotPose(pose);
        }

        public FieldObject2d getFieldTargetPose() {
                return _fieldWidget.getObject("target pose");
        }

        public FieldObject2d getFieldPath() {
                return _fieldWidget.getObject("path");
        }

        public void setAllianceColor(boolean isRed) {
                _allianceBox.setBoolean(isRed);
        }

        public void setGyroHeading(Rotation2d heading) {
                _headingGyro.setDouble(heading.getDegrees());
        }

        public void setFrontAprilTagId(int id) {
                _frontApTagIdField.setInteger(id);
        }

        public void setRearAprilTagId(int id) {
                _rearApTagIdField.setInteger(id);
        }

        public void setRearAprilTagOffset(double offset) {
                _rearApTagOffsetDial.setDouble(offset);
        }

        public void setFrontPoseEstimationSwitch(boolean enabled) {
                _frontPoseEstimationSwitch.setBoolean(enabled);
        }

        public void setRearPoseEstimationSwitch(boolean enabled) {
                _rearPoseEstimationSwitch.setBoolean(enabled);
        }

        public boolean getFrontPoseEstimationSwitch() {
                return _frontPoseEstimationSwitch.getBoolean(false);
        }

        public boolean getRearPoseEstimationSwitch() {
                return _rearPoseEstimationSwitch.getBoolean(false);
        }

        public BooleanEvent getFrontPoseEstimationEvent() {
                if (_frontPoseEstimationEvent == null) {
                        _frontPoseEstimationEvent = new BooleanEvent(Robot.EventLoop,
                                        this::getFrontPoseEstimationSwitch)
                                        .debounce(0.2);
                }

                return _frontPoseEstimationEvent;
        }

        public BooleanEvent getRearPoseEstimationEvent() {
                if (_rearPoseEstimationEvent == null) {
                        _rearPoseEstimationEvent = new BooleanEvent(Robot.EventLoop, this::getRearPoseEstimationSwitch)
                                        .debounce(0.2);
                }

                return _rearPoseEstimationEvent;
        }
}
