package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Controls;
import frc.constants.Controls.GulikitButtons;
import frc.constants.Controls.NumpadButtons;
import frc.constants.Subsystems.PortNumbers;
import frc.utils.math.Algebra;
import frc.utils.math.Trig;
import java.util.HashMap;
import java.util.Map;

public class OI {
    CommandJoystick joystick;
    CommandXboxController controller;
    CommandJoystick numpad;

    public enum Bind {
        enterShootingMode, 
        startShooting,
        setHighGoal,
        setLowGoal, 
        climbHighGoal,
        climbLowGoal,
        climbZero, 
        stopShooting, manualTurretLeft, manualTurretRight, manualTurretSwitch, manualTurretDown, manualTurretUp, intake, manualSpeedUp, manualSpeedDown, flywheel,
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
        joystick = new CommandJoystick(PortNumbers.ControllerPort);
        controller = new CommandXboxController(0);
        numpad = new CommandJoystick(PortNumbers.NumpadPort);

        binds.put(Bind.enterShootingMode, controller.button(GulikitButtons.EnterShootingModeButton)); 
        binds.put(Bind.startShooting, controller.button(GulikitButtons.StartShootingButton));
        binds.put(Bind.manualTurretLeft, controller.povLeft());
        binds.put(Bind.manualTurretRight, controller.povRight());  
        binds.put(Bind.manualTurretUp, controller.povUp());  
        binds.put(Bind.manualSpeedUp, controller.button(GulikitButtons.ManualSpeedUp));  
        binds.put(Bind.manualSpeedDown, controller.rightTrigger());  
         binds.put(Bind.manualTurretDown, controller.povDown()); 
          binds.put(Bind.manualTurretSwitch, controller.button(GulikitButtons.manualTurretSwitch)); 
        binds.put(Bind.setHighGoal, controller.button(GulikitButtons.setHighGoal)); 
        binds.put(Bind.setLowGoal, controller.button(GulikitButtons.setLowGoal)); 
        binds.put(Bind.climbHighGoal, controller.button(GulikitButtons.setHighGoal)); 
        binds.put(Bind.climbLowGoal, controller.button(GulikitButtons.setLowGoal)); 
        binds.put(Bind.climbZero, controller.button(GulikitButtons.setZeroGoal)); 
        binds.put(Bind.intake, controller.button(GulikitButtons.intake)); 
        binds.put(Bind.flywheel, controller.button(GulikitButtons.flywheel)); 

        //binds.put(Bind._, controller.button(_)); OR
        //binds.put(Bind._, numpad.button(_)); OR
    }

    private double deadbandCompensation(double r) {
        return (r - Controls.Deadband) / (1 - Controls.Deadband);
    }

    private double minimumPowerCompensation(double r) {
        return r * (1 - Controls.MinimumDrivePower) + Controls.MinimumDrivePower;
    }

    public double[] getXY(boolean isFlight) {
        double x = 0;
        double y = 0;
        double newX, newY = 0.0d;

        if (isFlight) {
        x = MathUtil.applyDeadband(joystick.getX(), Controls.Deadband);
        y = MathUtil.applyDeadband(joystick.getY(), Controls.Deadband);
        newX = 0.0d;
        newY = 0.0d;
        } else {
        x = MathUtil.applyDeadband(controller.getRawAxis(0), Controls.Deadband);
        y = MathUtil.applyDeadband(controller.getRawAxis(1), Controls.Deadband);
        }

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

    public double getRotation(boolean isFlight) {
       double rotation = 0;
        if (isFlight) {
             rotation = MathUtil.applyDeadband(controller.getRawAxis(5), Controls.Deadband);
        } else {
             rotation = MathUtil.applyDeadband(controller.getRawAxis(4), Controls.Deadband);
        }

        if (Controls.UseCurve) {
            rotation = Math.pow(minimumPowerCompensation(rotation), Controls.CurveExponent);
        } else {
            rotation = minimumPowerCompensation(rotation);
        }
        return rotation;
    }
}