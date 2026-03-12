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

import frc.subsystems.shooter.Shooter; 

public class OI {
    CommandJoystick controller;
    CommandJoystick numpad;

    public enum Bind {
        enterShootingMode, 
        startShooting,
        setHighGoal,
        setLowGoal, 
        climbHighGoal,
        climbLowGoal,
        climbZero,
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
        controller = new CommandJoystick(4);
        numpad = new CommandJoystick(1);

        binds.put(Bind.enterShootingMode, controller.button(GulikitButtons.EnterShootingModeButton)); 
        binds.put(Bind.startShooting, controller.button(GulikitButtons.ShootButtons)); 
        binds.put(Bind.setHighGoal, controller.button(GulikitButtons.setHighGoal)); 
        binds.put(Bind.setLowGoal, controller.button(GulikitButtons.setLowGoal)); 
        binds.put(Bind.climbHighGoal, controller.button(GulikitButtons.setHighGoal)); 
        binds.put(Bind.climbLowGoal, controller.button(GulikitButtons.setLowGoal)); 
        binds.put(Bind.climbZero, controller.button(GulikitButtons.setZeroGoal)); 

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
        double x = MathUtil.applyDeadband(controller.getX(), Controls.Deadband);
        double y = MathUtil.applyDeadband(controller.getY(), Controls.Deadband);
        return new double[] { x, y };
    }

    public double[] getRawXY() {
        return new double[] { controller.getRawAxis(GulikitButtons.LeftJoystickX),
            controller.getRawAxis(GulikitButtons.LeftJoystickY) };
    }

    public double getRotation() {
        double rotation = MathUtil.applyDeadband(controller.getRawAxis(5), Controls.Deadband);
        return rotation;
    }
}