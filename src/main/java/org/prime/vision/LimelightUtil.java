package org.prime.vision;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LimelightUtil {
    public static InterpolatingDoubleTreeMap StdDeviationAreaTreeMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(-10d, 999999d),
            Map.entry(0d, 20d),
            Map.entry(1d, 1d),
            Map.entry(5d, 0.1d),
            Map.entry(10d, 0.01d));
}
