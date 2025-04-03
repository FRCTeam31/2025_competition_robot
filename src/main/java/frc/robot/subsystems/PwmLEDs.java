package frc.robot.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elastic;

public class PwmLEDs extends SubsystemBase {
    public static class LEDMap {
        public static final int PwmPort = 9;
        public static final int PixelsPerSection = 8;
        public static final int SectionCount = 4;
        public static final int TotalPixels = PixelsPerSection * SectionCount;
        public static final int MaxLoopErrorsBeforeShutdown = 3;
    }

    private final ScheduledExecutorService _updateLoopExecutor = Executors.newScheduledThreadPool(1);
    private AddressableLED _led;
    private AddressableLEDBuffer _ledBuffer;
    private AddressableLEDBufferView[] _sections = new AddressableLEDBufferView[LEDMap.SectionCount];
    private LEDPattern[] _sectionPatterns = new LEDPattern[LEDMap.SectionCount];
    public byte _loopErrorCounter = 0;

    private LEDPattern _initialPattern = LEDPattern.solid(Color.kGhostWhite).breathe(Units.Seconds.of(4));
    private Alert _loopStoppedAlert;

    public PwmLEDs() {
        // Initialize the LED strip and buffer
        _ledBuffer = new AddressableLEDBuffer(LEDMap.TotalPixels);
        for (var i = 0; i < LEDMap.SectionCount; i++) {
            var startingIndex = i * LEDMap.PixelsPerSection;
            var endingIndex = startingIndex + LEDMap.PixelsPerSection - 1; // minus 1  here because the starting and ending indexes are inclusive
            _sections[i] = _ledBuffer.createView(startingIndex, endingIndex);
            _sectionPatterns[i] = _initialPattern;
        }

        _led = new AddressableLED(LEDMap.PwmPort);
        _led.setLength(_ledBuffer.getLength());
        _led.start();

        // LED update loop running at 125fps
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLedStrip, 0, 8, java.util.concurrent.TimeUnit.MILLISECONDS);

        // Apply a default pattern to the LED strip
        _initialPattern.applyTo(_ledBuffer);

        // Setup the warning for when the loop stops
        _loopStoppedAlert = new Alert("[LEDs] Update loop failed.", Alert.AlertType.kWarning);
        _loopStoppedAlert.set(false);
    }

    /**
     * Stops the update loop for the LEDs.
     */
    public void stopUpdateLoop() {
        _updateLoopExecutor.shutdown();
        _loopStoppedAlert.set(true);
    }

    /**
     * Starts the update loop for the LEDs.
     */
    public void startUpdateLoop() {
        _loopErrorCounter = 0;
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLedStrip, 0, 8, java.util.concurrent.TimeUnit.MILLISECONDS);
    }

    /**
     * Updates the LED strip with the current patterns.
     */
    private void updateLedStrip() {
        // If we've failed too many times, stop the loop and alert the user
        if (_loopErrorCounter > LEDMap.MaxLoopErrorsBeforeShutdown) {
            Elastic.sendError("LEDs Failed", "Update loop has failed 3 times. Stopping loop.");
            stopUpdateLoop();
        }

        try {
            for (var i = 0; i < LEDMap.SectionCount; i++) {
                _sectionPatterns[i].applyTo(_sections[i]);
            }

            // Update the LED strip with the new data
            _led.setData(_ledBuffer);
        } catch (Exception e) {
            // If we fail to update the LEDs, report the error and increment the error counter
            _loopErrorCounter++;
            DataLogManager.log("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage());
            DriverStation.reportError("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage(),
                    e.getStackTrace());
        }
    }

    public void setSectionPattern(int section, LEDPattern pattern) {
        _sectionPatterns[section] = pattern;
    }

    public void setAllSectionPatterns(LEDPattern pattern) {
        for (var i = 0; i < LEDMap.SectionCount; i++) {
            _sectionPatterns[i] = pattern;
        }
    }

    public Command setSectionPatternCommand(int section, LEDPattern pattern) {
        return Commands.runOnce(() -> setSectionPattern(section, pattern)).ignoringDisable(true);
    }

    public Command setAllSectionPatternsCommand(LEDPattern pattern) {
        return Commands.runOnce(() -> setAllSectionPatterns(pattern)).ignoringDisable(true);
    }
}
