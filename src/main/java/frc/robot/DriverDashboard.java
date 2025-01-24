package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class DriverDashboard {

    public static ShuffleboardTab DriverTab = Shuffleboard.getTab("Driver");
    public static ShuffleboardTab AutoTab = Shuffleboard.getTab("Auto Commands");

    public static SendableChooser<Command> AutoChooser;
    public static GenericEntry AllianceBox = DriverTab
            .add("Alliance", false)
            .withPosition(15, 0)
            .withSize(2, 3)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#0000FF"))
            .getEntry();

    // Drive
    public static Field2d FieldWidget = new Field2d();
    public static GenericEntry HeadingGyro = DriverTab
            .add("Current Heading", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(12, 0)
            .withSize(3, 3)
            .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
            .getEntry();
    public static GenericEntry RearApTagIdField = DriverTab
            .add("Rear APTag", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(12, 3)
            .withSize(2, 1)
            .getEntry();
    public static GenericEntry FrontApTagIdField = DriverTab
            .add("Front APTag", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(14, 3)
            .withSize(2, 1)
            .getEntry();
    public static GenericEntry RearApTagOffsetDial = DriverTab
            .add("Rear APTag X Offset", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", -29.8, "Max", 29.8))
            .withPosition(12, 4)
            .withSize(2, 3)
            .getEntry();
    public static GenericEntry FrontPoseEstimationSwitch = DriverTab
            .add("F Pose Est.", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(14, 4)
            .withSize(2, 1)
            .getEntry();
    public static GenericEntry RearPoseEstimationSwitch = DriverTab
            .add("R Pose Est.", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(14, 5)
            .withSize(2, 1)
            .getEntry();

    /**
     * Constructs a new DriverDashboard and adds complex widgets that must be created in the constructor
     * @param config
     */
    public static void initialize(boolean isReal) {
        // DriverTab
        //   .addCamera(
        //     "Rear Limelight",
        //     "Limelight Rear",
        //     "http://" + config.Drivetrain.LimelightRearName + ".local:5800/stream.mjpg"
        //   )
        //   .withPosition(0, 0)
        //   .withSize(6, 6)
        //   .withWidget(BuiltInWidgets.kCameraStream)
        //   .withProperties(Map.of("Show controls", false, "Show crosshair", false));

        DriverTab.add("Field", FieldWidget).withWidget(BuiltInWidgets.kField).withPosition(0, 0).withSize(12, 6);
    }

    /**
     * Configures the autonomous dashboard items
     */
    public static void setupAutonomousDashboardItems() {
        // Build an auto chooser for the Driver tab. This will use Commands.none() as the default option.
        AutoChooser = AutoBuilder.buildAutoChooser("Default");
        DriverTab.add(AutoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 6).withSize(5, 2);

        // Add all autos to the Auto tab for testing
        var possibleAutos = AutoBuilder.getAllAutoNames();
        for (int i = 0; i < possibleAutos.size(); i++) {
            var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
            DriverDashboard.AutoTab.add(possibleAutos.get(i), autoCommand)
                    .withWidget(BuiltInWidgets.kCommand).withSize(2, 1);
        }
    }
}
