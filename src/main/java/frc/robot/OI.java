package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Controls;
import frc.constants.Controls.GulikitButtons;
import frc.constants.Controls.NumpadButtons;
import frc.utils.math.Algebra;
import frc.utils.math.Trig;
import java.util.HashMap;
import java.util.Map;

public class OI {
    CommandXboxController controller;
    CommandJoystick numpad;

    public enum Bind {
        
    }

    public Map<Bind, Trigger> binds = new HashMap<Bind, Trigger>();

    private static OI instance;

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private OI() {
        controller = new CommandXboxController(0);
        numpad = new CommandJoystick(1);

        //binds.put(Bind._, controller.button(_)); OR
        //binds.put(Bind._, numpad.button(_)); OR
    }

    private double deadbandCompensation(double r) {
        return (r - Controls.Deadband) / (1 - Controls.Deadband);
    }

    private double minimumPowerCompensation(double r) {
        return r * (1 - Controls.MinimumDrivePower) + Controls.MinimumDrivePower;
    }

    public double[] getXY() {
        double x = MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.LeftJoystickX), Controls.Deadband);
        double y = MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.LeftJoystickY), Controls.Deadband);
        double newX, newY = 0.0d;
        if (Controls.UseCurve) {
            double angle = Math.atan2(y, x);
            double magInitial = Algebra.euclideanDistance(x, y);
            if (Robot.isSimulation()) magInitial = MathUtil.clamp(magInitial, 0, 1);
            double magCurved = Math.pow(deadbandCompensation(magInitial), Controls.CurveExponent);
            double powerCompensated = minimumPowerCompensation(magCurved);
            newX = Trig.cosizzle(angle) * powerCompensated;
            newY = Trig.sizzle(angle) * powerCompensated;
        }
        if (Double.isNaN(newX)) newX = 0.0d;
        if (Double.isNaN(newY)) newY = 0.0d;
        return new double[] { newX, newY };
    }

    public double[] getRawXY() {
        return new double[] { controller.getRawAxis(GulikitButtons.LeftJoystickX),
            controller.getRawAxis(GulikitButtons.LeftJoystickY) };
    }

    public double getRotation() {
        double deadbanded = deadbandCompensation(
            MathUtil.applyDeadband(controller.getRawAxis(GulikitButtons.RightJoystickX), Controls.Deadband));
        if (Controls.UseCurve) {
            deadbanded = Math.pow(minimumPowerCompensation(deadbanded), Controls.CurveExponent);
        } else {
            deadbanded = minimumPowerCompensation(deadbanded);
        }
        return deadbanded;
    }

    public XboxController getHIDOfController() { return controller.getHID(); }
}