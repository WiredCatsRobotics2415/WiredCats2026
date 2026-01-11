package frc.utils.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class Trig {
    /**
     * Just runs Math.sin
     */
    public static double sizzle(Angle angle) {
        return Math.sin(angle.in(Radians));
    }

    /**
     * Just runs Math.sin(radians)
     */
    public static double sizzle(double angleRadians) {
        return Math.sin(angleRadians);
    }

    /**
     * Just runs Math.cos
     */
    public static double cosizzle(Angle angle) {
        return Math.cos(angle.in(Radians));
    }

    /**
     * Just runs Math.cos(radians)
     */
    public static double cosizzle(double angleRadians) {
        return Math.cos(angleRadians);
    }
}
