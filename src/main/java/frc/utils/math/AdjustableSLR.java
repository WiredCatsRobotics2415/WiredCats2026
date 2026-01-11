package frc.utils.math;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import lombok.Setter;

public class AdjustableSLR {
    @Setter private double rateLimit;
    private double prevValue;
    private double prevTime;

    public AdjustableSLR(double rateLimit, double initialValue) {
        this.rateLimit = rateLimit;
        this.prevValue = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public AdjustableSLR(double rateLimit) {
        this(rateLimit, 0);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        prevValue += MathUtil.clamp(input - prevValue, -rateLimit * elapsedTime, rateLimit * elapsedTime);
        prevTime = currentTime;
        return prevValue;
    }
}
