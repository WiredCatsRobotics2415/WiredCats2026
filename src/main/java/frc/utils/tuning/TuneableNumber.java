package frc.utils.tuning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.constants.RuntimeConstants;
import java.util.ArrayList;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuneableNumber {
    private static ArrayList<TuneableNumber> all = new ArrayList<TuneableNumber>();
    private static int indexToUpdate = 0;
    private String key;

    private Angle lastAngle;
    private Distance lastDistance;

    private LoggedNetworkNumber thisNetworkNumber;
    private double previousNumber;
    private ArrayList<Consumer<Double>> listeners;

    public TuneableNumber(double defaultNumber, String key) {
        this.previousNumber = defaultNumber;
        this.key = key;
        if (RuntimeConstants.TuningMode) {
            thisNetworkNumber = new LoggedNetworkNumber("/Tuning/" + key, defaultNumber);
            thisNetworkNumber.setDefault(defaultNumber);
            listeners = new ArrayList<Consumer<Double>>();
            all.add(this);
        }
    }

    public TuneableNumber(Distance defaultDistance, String key) {
        this(defaultDistance.in(Inches), key);
        lastDistance = defaultDistance;
    }

    public TuneableNumber(Angle defaultAngle, String key) {
        this(defaultAngle.in(Degrees), key);
        lastAngle = defaultAngle;
    }

    public double get() {
        if (RuntimeConstants.TuningMode && listeners.size() == 0) return thisNetworkNumber.get();
        return previousNumber;
    }

    /**
     * Assuming this number is a distance in inches, convert it to meters
     */
    public double meters() {
        return Units.inchesToMeters(get());
    }

    /**
     * Assuming this number is in inches
     */
    public Distance distance() {
        return Inches.of(get());
    }

    /**
     * Assuming this number is an angle in degrees, convert it to radians
     */
    public double radians() {
        return Units.degreesToRadians(get());
    }

    /**
     * Assuming this number is in degrees
     */
    public Angle angle() {
        return Degrees.of(get());
    }

    public Rotation2d rotation() {
        return Rotation2d.fromDegrees(get());
    }

    public void set(double newNumber) {
        previousNumber = newNumber;
        if (RuntimeConstants.TuningMode) {
            thisNetworkNumber.set(newNumber);
        }
    }

    /**
     * Consumer to call when value is changed from NT. Do NOT call get() on this tuneable from within the consumer, because the value will not have changed yet.
     *
     * @param toCall
     */
    public void addListener(Consumer<Double> toCall) {
        listeners.add(toCall);
    }

    private void updateFromNT() {
        if (listeners.size() == 0) return;
        double entry = thisNetworkNumber.get();
        if (entry != previousNumber) {
            previousNumber = entry;
            if (lastAngle != null) lastAngle = Degrees.of(previousNumber);
            if (lastDistance != null) lastDistance = Inches.of(previousNumber);
            for (Consumer<Double> r : listeners)
                r.accept(entry);
        }
    }

    public static void periodic() {
        all.get(indexToUpdate).updateFromNT();
        indexToUpdate += 1;
        if (indexToUpdate == all.size()) {
            indexToUpdate = 0;
        }
    }
}