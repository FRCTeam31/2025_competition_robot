package frc.robot.oi;

import java.util.LinkedList;
import java.util.Queue;

public class ImpactRumbleHelper {
    private static final int SAMPLE_WINDOW = 50; // 1 second (50 samples at 20ms intervals)
    private static final double IMPACT_THRESHOLD_G = 0.5; // Threshold for detecting an impact
    private static final double SCALING_FACTOR = 0.5; // Scales impact strength to rumble intensity
    private static final double MIN_VELOCITY_THRESHOLD = 0.1; // Ignore impacts when nearly stationary
    private static final int TAPER_OFF_DURATION = 25; // 0.5 seconds (25 samples)

    private final Queue<Double> accelMagnitudes = new LinkedList<>();
    private final Queue<Double> velocitySamples = new LinkedList<>();

    private double previousAccelMagnitude = 0;
    private double currentRumbleIntensity = 0;
    private int taperOffCounter = 0;

    public void addSample(double accelX, double accelY, double accelZ, double velocity, double maxVelocity) {
        // Compute acceleration magnitude
        double accelMagnitude = Math.sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

        // Compute acceleration change
        double deltaA = Math.abs(accelMagnitude - previousAccelMagnitude);
        previousAccelMagnitude = accelMagnitude;

        // Maintain rolling buffer of samples
        accelMagnitudes.add(accelMagnitude);
        velocitySamples.add(velocity);
        if (accelMagnitudes.size() > SAMPLE_WINDOW)
            accelMagnitudes.poll();
        if (velocitySamples.size() > SAMPLE_WINDOW)
            velocitySamples.poll();

        // Check if impact exceeds threshold
        if (deltaA > IMPACT_THRESHOLD_G) {
            double rumbleIntensity = Math.min(1.0, SCALING_FACTOR * deltaA);

            // Reduce false positives at low speed
            if (velocity < MIN_VELOCITY_THRESHOLD * maxVelocity) {
                rumbleIntensity *= 0.5;
            }

            // Update rumble intensity
            currentRumbleIntensity = rumbleIntensity;
            taperOffCounter = TAPER_OFF_DURATION * (int) rumbleIntensity; // Reset taper-off timer and scale it to intensity
        }
    }

    public double getRumbleIntensity() {
        // If no new impact, taper off intensity over time
        if (taperOffCounter > 0) {
            taperOffCounter--;
            currentRumbleIntensity *= 0.96; // Gradual fade-out
        } else {
            currentRumbleIntensity = 0;
        }
        return currentRumbleIntensity;
    }
}