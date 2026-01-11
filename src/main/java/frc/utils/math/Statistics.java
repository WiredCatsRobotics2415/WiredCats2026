package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import java.util.List;
import java.util.stream.DoubleStream;

public class Statistics {
    /**
     * Gets the length of a list or an array.
     *
     * @param obj the list or array, throws IllegalArgumentException if obj is not a list or array
     * @return the length
     */
    private static int getLength(Object obj) {
        if (obj instanceof List<?>) {
            return ((List<?>) obj).size();
        } else if (obj.getClass().isArray()) {
            return java.lang.reflect.Array.getLength(obj);
        }
        throw new IllegalArgumentException("Unsupported type");
    }

    public static double getPercentile(double[] sortedData, double percentile) {
        double index = (percentile / 100.0) * (sortedData.length - 1);
        int lower = (int) Math.floor(index);
        int upper = (int) Math.ceil(index);

        if (lower == upper) {
            return sortedData[lower];
        }

        return sortedData[lower] + (index - lower) * (sortedData[upper] - sortedData[lower]);
    }

    public static double[] removeOutliersIQR(double[] data) {
        Arrays.sort(data);
        double q1 = getPercentile(data, 25);
        double q3 = getPercentile(data, 75);
        double iqr = q3 - q1;
        int outOfIQR = 0;
        for (double d : data) {
            if (d < q3 - (1.5 * iqr) || d > q1 + (1.5 * iqr)) outOfIQR++;
        }
        double[] filtered = new double[data.length - outOfIQR];
        int i = 0;
        for (double d : data) {
            boolean outlier = (d < q3 - (1.5 * iqr) || d > q1 + (1.5 * iqr));
            if (outlier) continue;
            filtered[i] = d;
            i++;
        }
        return filtered;
    }

    /**
     * Preforms a circular mean on a set of rotation2ds
     *
     * @param data           an iteratable of rotation2ds
     * @param removeOutliers whether or not to remove outliers based on 1.5 IQR
     * @return the rotation2d of the circular mean
     */
    public static Rotation2d circularMean(Iterable<Rotation2d> data) {
        double sumX = 0.0d, sumY = 0.0d;
        int i = 0;
        for (Rotation2d d : data) {
            sumX += d.getCos();
            sumY += d.getSin();
            i++;
        }
        double meanX = sumX / i, meanY = sumY / i;
        return Rotation2d.fromRadians(Math.atan2(meanY, meanX));
    }

    public static Rotation2d circularMeanRemoveOutliers(Iterable<Rotation2d> data, int listLength) {
        double[] cosValues = new double[listLength], sinValues = new double[listLength];
        int i = 0;
        for (Rotation2d d : data) {
            cosValues[i] = d.getCos();
            sinValues[i] = d.getSin();
            i++;
        }
        cosValues = removeOutliersIQR(cosValues);
        sinValues = removeOutliersIQR(sinValues);

        double meanX = DoubleStream.of(cosValues).average().getAsDouble(),
            meanY = DoubleStream.of(sinValues).average().getAsDouble();
        return Rotation2d.fromRadians(Math.atan2(meanY, meanX));
    }
}
