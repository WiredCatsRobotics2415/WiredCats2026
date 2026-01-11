package frc.utils.math;

public class Algebra {
    /**
     * Changes the range of an input linearly, exactly like Arduino's "map()" function
     *
     * @param inputMin  minimum (lowest number on number line) input can take
     * @param inputMax  maximum (highest number on number line) input can take
     * @param outputMin min value of output
     * @param outputMax max value of output
     */
    public static double linearMap(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
    }

    /**
     * multiples the arrays element wise
     */
    public static double addWeighted(double[] weights, double[] values) {
        if (weights.length != values.length) {
            throw new IllegalArgumentException("Weights and values must be the same length, but weights has "
                + weights.length + " elements and values has " + values.length);
        }
        double sum = 0.0d;
        for (int i = 0; i < weights.length; i++) {
            sum += weights[i] * values[i];
        }
        return sum;
    }

    public static double euclideanDistance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
}
