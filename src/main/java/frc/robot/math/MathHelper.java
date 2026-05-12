package frc.robot.math;

import frc.robot.subsystems.network.NetworkSubsystem;

/**
 * Utility class providing non-linear scaling functions for controller inputs.
 * Applies power curves to reduce sensitivity at low inputs while maintaining high-end precision.
 */
public class MathHelper {
    /**
     * Scales speed input with a power curve (exponent 2.6) for finer control at low speeds.
     * 
     * @param input Raw input value (-1.0 to 1.0)
     * @return Scaled output value (-1.0 to 1.0) with preserved sign
     */
    public static double ScaleSpeedInput(double input) {
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        double scaledInput = Math.pow(absInput, NetworkSubsystem.SPEED_CURVE.get());

        return sign * scaledInput;
    }
    
    /**
     * Scales rotation input with a power curve (exponent 1.8) for smoother turning.
     * 
     * @param input Raw input value (-1.0 to 1.0)
     * @return Scaled output value (-1.0 to 1.0) with preserved sign
     */
    public static double ScaleRotInput(double input) {
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        double scaledInput = Math.pow(absInput, 1.8);

        return sign * scaledInput;
    }

    public static double ScaleLaserInput(double input) {
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        double scaledInput = Math.pow(absInput, 2.0);

        return sign * scaledInput;
    }
    
    /**
     * Linearly interpolates between two values based on a rate parameter.
     * 
     * @param min Minimum value (returned when rate <= 0)
     * @param max Maximum value (returned when rate >= 1)
     * @param rate Interpolation factor (0.0 to 1.0)
     * @return Interpolated value between min and max
     */
    public static double Interpolate(double min, double max, double rate) {
        if (rate < 0) return min;
        if (rate > 1) return max;

        return min + (max - min) * rate;
    }
}
