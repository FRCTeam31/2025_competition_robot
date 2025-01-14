package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class PwmLEDs extends SubsystemBase {
    public static class VMap {
        public static final int PwmPort = 9;
        public static final int PixelsPerStrip = 78;
        public static final double BackgroundDimAmount = 0.5;
    }

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    @Logged(name = "LoopErrorCounter", importance = Logged.Importance.CRITICAL)
    public byte _loopErrorCounter = 0;

    private LEDPattern m_backgroundPattern = LEDPattern.solid(Color.kGhostWhite)
        .breathe(Units.Seconds.of(4));
    private LEDPattern m_foregroundPattern = null;

    private Alert m_loopStoppedAlert;
    private Dimensionless m_backgroundDimAmount = Units.Percent.of(50);

    public PwmLEDs() {
        // Initialize the LED strip and buffer
        m_ledBuffer = new AddressableLEDBuffer(VMap.PixelsPerStrip);
        m_led = new AddressableLED(VMap.PwmPort);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();

        // Apply a default pattern to the LED strip
        m_backgroundPattern.applyTo(m_ledBuffer);

        // Setup the warning for when the loop stops
        m_loopStoppedAlert = new Alert("[LEDs:ERROR] LED update loop failed.", Alert.AlertType.kWarning);
        m_loopStoppedAlert.set(false);
    }

    public void updateLedStrip() {
        // If we're not running on a real robot, or we've failed too many times, do nothing.
        if (_loopErrorCounter > 3)
            return;
            
        try {
            if (m_foregroundPattern == null) {
                // If we're only using a background pattern, apply it directly
                m_backgroundPattern.applyTo(m_ledBuffer);
            } else {
                // If we have a foreground pattern, overlay it on a dimmed background
                var dimmedBackground = m_backgroundPattern.atBrightness(m_backgroundDimAmount);
                m_foregroundPattern.overlayOn(dimmedBackground).applyTo(m_ledBuffer);
            }

            // Update the LED strip with the new data
            m_led.setData(m_ledBuffer);
        } catch (Exception e) {
            // If we fail to update the LEDs, report the error and increment the error counter
            _loopErrorCounter++;
            DataLogManager.log("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage());
            DriverStation.reportError("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage(), e.getStackTrace());

            // If we've failed too many times, stop the loop and alert the user
            if (_loopErrorCounter > 3) {
                var msg = "[LEDs:ERROR] LED update loop has failed 3 times. Stopping loop.";
                DriverStation.reportError(msg, false);
                System.out.println(msg);
                m_loopStoppedAlert.set(true);
            }
        }
    }

    public void setBackgroundPattern(LEDPattern backgroundPattern) {
        if (m_backgroundPattern != backgroundPattern) {
            m_backgroundPattern = backgroundPattern;
        }
    }

    public void setForegroundPattern(LEDPattern foregroundPattern) {
        if (DriverStation.isEnabled() && m_foregroundPattern != foregroundPattern) {
            m_foregroundPattern = foregroundPattern;
        }
    }

    public void clearForegroundPattern() {
        if (m_foregroundPattern != null)
            m_foregroundPattern = null;
    }

    public Command setBackgroundPatternCommand(LEDPattern backgroundPattern) {
        return runOnce(() -> setBackgroundPattern(backgroundPattern))
            .ignoringDisable(true);
    }

    public Command setForegroundPatternCommand(LEDPattern foregroundPattern) {
        return runOnce(() -> setForegroundPattern(foregroundPattern))
            .ignoringDisable(true); // (only allowed when enabled)
    }

    public Command clearForegroundPatternCommand() {
        return runOnce(() -> clearForegroundPattern()).ignoringDisable(true);
    }
}
