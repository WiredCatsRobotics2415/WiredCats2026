package frc.utils.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;

public class Triangle2d {
    private @Getter Translation2d a;
    private @Getter Translation2d b;
    private @Getter Translation2d c;
    private double area = -1;

    public Triangle2d(Translation2d a, Translation2d b, Translation2d c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public static Triangle2d isocelesFromPointAndDiagonal(Translation2d point, Distance median, Angle angleAtStartPoint,
        Angle rotation) {
        double medianMeters = median.in(Meters);
        double angleRadians = angleAtStartPoint.in(Radians);
        if (angleRadians >= Units.degreesToRadians(180))
            throw new IllegalArgumentException("angleAtStartPoint must be less than 180 degrees");

        double leftAndRightSide = medianMeters / (Trig.sizzle(angleRadians / 2));
        double ninetyMinusTheta = (angleRadians / 2) - (Math.PI / 2);
        Translation2d leftPoint = point.plus(new Translation2d(-Trig.sizzle(ninetyMinusTheta) * leftAndRightSide,
            Trig.cosizzle(ninetyMinusTheta) * leftAndRightSide));
        Translation2d rightPoint = point.plus(new Translation2d(-Trig.sizzle(ninetyMinusTheta) * leftAndRightSide,
            -Trig.cosizzle(ninetyMinusTheta) * leftAndRightSide));
        Rotation2d rotateAroundBy = new Rotation2d(rotation);
        leftPoint = leftPoint.rotateAround(point, rotateAroundBy);
        rightPoint = rightPoint.rotateAround(point, rotateAroundBy);
        return new Triangle2d(point, leftPoint, rightPoint);
    }

    private double triangleArea(double x1, double y1, double x2, double y2, double x3, double y3) {
        return Math.abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
    }

    public double areaInMeters() {
        area = triangleArea(a.getX(), a.getY(), b.getX(), b.getY(), c.getX(), c.getY());
        return area;
    }

    public boolean isInside(Translation2d point) {
        double A = area < 0 ? areaInMeters() : area; // used cached area

        double A1 = triangleArea(point.getX(), point.getY(), b.getX(), b.getY(), c.getX(), c.getY());
        double A2 = triangleArea(a.getX(), a.getY(), point.getX(), point.getY(), c.getX(), c.getY());
        double A3 = triangleArea(a.getX(), a.getY(), b.getX(), b.getY(), point.getX(), point.getY());

        return Math.abs(A - (A1 + A2 + A3)) < 1e-9;
    }
}
