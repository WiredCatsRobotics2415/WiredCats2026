package frc.constants;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {
    //public static final double MaxDriveMeterS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxDriveMeterS=1;
    public static final double MaxAngularRadS = 1.2 * Math.PI;
    public static final double Deadband = 0.05d;
    public static final double MinimumDrivePower = 0.05d;
    public static final boolean UseCurve = true;
    public static final double CurveExponent = 3;

    public static final double RumbleSoftValue = 0.2d;
    public static final double RumbleHardValue = 0.6d;

    public class GulikitButtons {
        public static final int X = 4;
        public static final int Y = 3;
        public static final int B = 1;
        public static final int A = 2;
        public static final int Plus = XboxController.Button.kStart.value;
        public static final int Minus = XboxController.Button.kBack.value;
        public static final int LeftBumper = XboxController.Button.kLeftBumper.value;
        public static final int RightBumper = XboxController.Button.kRightBumper.value;
        public static final int LeftPaddle = 9;
        public static final int RightPaddle = 10;

        public static final int LeftJoystickX = XboxController.Axis.kLeftX.value;
        public static final int LeftJoystickY = XboxController.Axis.kLeftY.value;
        public static final int RightJoystickX = XboxController.Axis.kRightX.value;
        public static final int RightJoystickY = XboxController.Axis.kRightY.value;
        public static final int LeftTrigger = XboxController.Axis.kLeftTrigger.value;
        public static final int RightTrigger = XboxController.Axis.kRightTrigger.value;

        //define buttons!!!
        public static final int EnterShootingModeButton = 1; 
        public static final int StartShootingButton = 2; 
        public static final int setHighGoal = 80;
        public static final int setLowGoal = 81;
        public static final int setZeroGoal = 82;
        public static final int ManualTurretLeft = 9;
        public static final int ManualTurretRight = 7;
        public static final int ManualTurretUp = 8;
        public static final int ManualTurretDown = 6;
        public static final int ManualSpeedUp = 11;
        public static final int ManualSpeedDown = 13;
        public static int manualTurretSwitch = 10;
        public static int intake = 3;
    }

    public class NumpadButtons {
        public static final int NumberZero = 1;
        public static final int Dot = 2;
        public static final int NumberOne = 3;
        public static final int NumberTwo = 4;
        public static final int NumberThree = 5;
        public static final int NumberFour = 6;
        public static final int NumberFive = 7;
        public static final int NumberSix = 8;
        public static final int NumberSeven = 9;
        public static final int NumberEight = 10;
        public static final int NumberNine = 11;
        public static final int ForwardSlash = 12;
        public static final int Asterisk = 13;
        public static final int Minus = 14;
        public static final int Plus = 15;
    }
}

