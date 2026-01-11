package frc.utils.math;

public class Point2d {
    private double x;
    private double y;

    public Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }
}
