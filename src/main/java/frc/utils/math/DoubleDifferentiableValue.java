package frc.utils.math;

import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class DoubleDifferentiableValue {
    @Getter private double value;
    @Getter private double firstDerivative;
    @Getter private double secondDerivative;

    private boolean firstUpdate = true;
    private boolean secondUpdate = false;
    private double lastTimestamp;
    private double lastVelocity;

    public DoubleDifferentiableValue() {}

    /**
     * Updates the value and its derivatives using Timer.getFPGATimestamp. Note that accuracy depends on the magnitude of the actual change in the quantity, and that the higher order we differentiate, the less accurate we will be.
     *
     * @param update
     */
    public void update(double update) {
        double nowTimestamp = Timer.getFPGATimestamp();
        if (firstUpdate) {
            value = update;
            firstDerivative = 0.0d;
            secondDerivative = 0.0d;
        } else {
            firstDerivative = (update - value) / (nowTimestamp - lastTimestamp);
            if (secondUpdate) {
                secondDerivative = 0.0d;
            } else {
                secondDerivative = (firstDerivative - lastVelocity) / (nowTimestamp - lastTimestamp);
            }
        }
        lastTimestamp = nowTimestamp;
        value = update;
        lastVelocity = firstDerivative;
        if (!firstUpdate && secondUpdate) secondUpdate = true;
        firstUpdate = false;
    }

    /**
     * Does first order linear approximation
     *
     * @param time the delta t that you want an approximation at
     * @return the approximation
     */
    public double firstDerivativeLinearApprox(double time) {
        return value + firstDerivative * time;
    }

    /**
     * Does linear approximation taking the 2nd derivative into account Note: Treat this with caution - the 2nd derivatives are unpredictable according to experience, so test before using this method for anything important.
     *
     * @param time the delta t that you want an approximation at
     * @return the approximation
     */
    public double secondDerivativeLinearApprox(double time) {
        return firstDerivativeLinearApprox(time) + 0.5 * secondDerivative * (time * time);
    }
}
