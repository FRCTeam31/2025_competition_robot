package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.oi.PrimeAutoRoutine;

import java.util.ArrayList;
import java.util.List;

import org.prime.dashboard.PrimeSendableChooser;

public class DriverDashboardTab extends DashboardSection {
        private final String _fieldName = "Field";
        private final Field2d _fieldWidget;
        private final String _autoBuilderName = "PrimeAutoBuilder";
        private PrimeAutoRoutine _autoBuilder;
        private final String _autoOptionChooserName = "Auto Options";
        private PrimeSendableChooser<String> _autoOptionChooser;

        // Widgets
        private final String _allianceBoxName = "Alliance";
        private final String _headingGyroName = "Current Heading";
        private final String _frontPoseEstimationSwitchName = "F Pose Est.";
        private BooleanEvent _frontPoseEstimationEvent = null;
        private final String _rearPoseEstimationSwitchName = "R Pose Est.";
        private BooleanEvent _rearPoseEstimationEvent = null;

        public DriverDashboardTab(PrimeAutoRoutine autoBuilder) {
                super("Driver");

                putBoolean(_allianceBoxName, false);
                putDouble(_headingGyroName, 0.0);
                putBoolean(_frontPoseEstimationSwitchName, false);
                putBoolean(_rearPoseEstimationSwitchName, false);

                // Add complex data like the field view
                _fieldWidget = new Field2d();
                putData(_fieldName, _fieldWidget);

                // _autoChooser = AutoBuilder.buildAutoChooser("Default");
                // _tab.add(_autoChooser)
                //                 .withWidget(BuiltInWidgets.kComboBoxChooser)
                //                 .withPosition(0, 6)
                //                 .withSize(5, 2);
                _autoOptionChooser = new PrimeSendableChooser<String>();
                putData(_autoOptionChooserName, _autoOptionChooser);

                autoBuilder.setChooser(_autoOptionChooser);
                _autoBuilder = autoBuilder;
                putData(_autoBuilderName, _autoBuilder);
        }

        public Command getSelectedAuto() {
                // return _autoChooser.getSelected();
                return _autoBuilder.exportCombinedAutoRoutine();
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

        public void setFieldPath(List<Pose2d> poses) {
                getFieldPath().setPoses(poses);
        }

        public void clearFieldPath() {
                getFieldPath().setPoses(new ArrayList<>());
        }

        public void setAllianceColor(boolean isRed) {
                putBoolean(_allianceBoxName, isRed);
        }

        public void setGyroHeading(Rotation2d heading) {
                putDouble(_headingGyroName, heading.getDegrees());
        }

        public void setFrontPoseEstimationSwitch(boolean enabled) {
                putBoolean(_frontPoseEstimationSwitchName, enabled);
        }

        public void setRearPoseEstimationSwitch(boolean enabled) {
                putBoolean(_rearPoseEstimationSwitchName, enabled);
        }

        public boolean getFrontPoseEstimationSwitch() {
                return getBoolean(_frontPoseEstimationSwitchName, false);
        }

        public boolean getRearPoseEstimationSwitch() {
                return getBoolean(_rearPoseEstimationSwitchName, false);
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
