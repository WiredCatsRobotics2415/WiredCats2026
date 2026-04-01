package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.intake.Intake;
import frc.subsystems.shooter.Shooter;
import frc.visualization.BallSim;
import frc.constants.Measurements;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import org.apache.commons.math3.util.MathArrays.Position;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import frc.constants.Subsystems.TurretConstants;
import frc.constants.Subsystems.PortNumbers; 
import frc.subsystems.turret.TurretSim; 
import edu.wpi.first.wpilibj.Servo;

public class Turret extends SubsystemBase {
    private static Turret instance; 
    private static Shooter shooter = Shooter.getInstance();
    private final Encoder encoder = new Encoder(PortNumbers.encoderChannelA, PortNumbers.encoderChannelB);
    public double currentAngle = 0.0;
    public double currentPosition = 0.5;
    public Servo turret1;
    public Servo turret2;
    public AnalogInput turret1analog;
    public AnalogInput turret2analog;
    private boolean currentlyUp = false;
    public double totalEncoderDisplacement = 17.43;
    public double startingTurret1Pos=490;
    public double startingTurret2Pos=490;

    private DigitalInput leftLimitSwitch;
    private DigitalInput rightLimitSwitch;

    public TurretSim sim = new TurretSim();

    private static TalonFX motor;
    final PositionVoltage m_request = new PositionVoltage(0);

    private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(TurretConstants.kMaxVelocity.get(), TurretConstants.kMaxAcceleration.get());
    private final ProfiledPIDController controller =
      new ProfiledPIDController(TurretConstants.kP.get(), TurretConstants.kI.get(), TurretConstants.kD.get(), constraints, 0.02);
    
    public static Turret getInstance() {
        motor = new TalonFX(PortNumbers.Turret_Motor);

        if (instance == null) instance = new Turret(); 
        return instance; 
    }

    public Turret() {
      turret1 = new Servo(PortNumbers.TurretServo1);
      turret2 = new Servo(PortNumbers.TurretServo2);
      turret1analog = new AnalogInput(PortNumbers.TurretServo1Analog);
      turret2analog = new AnalogInput(PortNumbers.TurretServo2Analog);

      leftLimitSwitch = new DigitalInput(PortNumbers.TurretLimit1);
      rightLimitSwitch = new DigitalInput(PortNumbers.TurretLimit2);

      encoder.reset();
      encoder.setDistancePerPulse(0.001);
    }

    public double setAngle(double angle) {
      if (angle < Measurements.MaxTurretAngle && angle > Measurements.MinTurretAngle) {
            currentAngle = angle;
            //angle from -180 to 180
            double position = ((angle + 180)/360 * totalEncoderDisplacement); // Convert from degrees to rotations
            if (position > 0 && position < totalEncoderDisplacement) {
              Logger.recordOutput("Turret/SettingPosition", position);
              return position;
            } else {
              return encoder.getDistance();
            }
      } else {
        return encoder.getDistance();
      }
    }

    public void setControllerChange(double posChange) {
      controller.setGoal(controller.getGoal().position + posChange);
      System.out.println(controller.getGoal().position + posChange);
    }

    public double mapPotentiometerToOne(double pot) {
      return Math.abs(1-(pot-373)/(1947));
    }
    
    public void IpswitchPitchSwitch() {
      if (currentlyUp==false) {
      //up for turret 1: 1651
      //down for turret 1: 1368
      //up for turret 2: 1112
      //down for turret 2: 1426
        turret1.set(mapPotentiometerToOne(1651));
        turret2.set(mapPotentiometerToOne(1112));
        currentlyUp = true;
      } else {
        turret1.set(mapPotentiometerToOne(1368));
        turret2.set(mapPotentiometerToOne(1426));
        currentlyUp= false;
      }
    }

    public double getPitchAngle() {
      return currentPosition;
    }

    public double getAngle() {
        return currentAngle;
    }

    public double relativeToTurretAngle(double angle) {
      return (angle + 90 + 360) % 360;
    }

    @Override
    public void periodic() {
      Logger.recordOutput("Turret/currrentPitch", currentPosition);
      Logger.recordOutput("Turret/controllerGoal", controller.getGoal().position);
      Logger.recordOutput("Turret/encoderConnected", encoder.get());
      Logger.recordOutput("Turret/encoderPos", encoder.getDistance());

      if ((turret1analog!=null)) {
        Logger.recordOutput("Turret/turret1", turret1analog.getValue());
      }
      if (turret2analog!=null) {
        Logger.recordOutput("Turret/turret2", turret2analog.getValue());
      }

      Logger.recordOutput("Turret/leftlimit", leftLimitSwitch.get());
      Logger.recordOutput("Turret/rightlimit", rightLimitSwitch.get());

      // turret1.set(turret1.get());
      // turret2.set(turret2.get());
    
        //get the angle from the shooter, convert to relative turret angle, then 
        //convert that to a 0-1 position, and set that as the controller goal for the turret
        double angle = shooter.getLastAngle();
        //double newAngle = relativeToTurretAngle(Math.toDegrees(angle));
        double newAngle = -160;
        Logger.recordOutput("Turret/AngleGoal", newAngle);
        double setting = 0;
        setting = setAngle(newAngle);

        if (RobotContainer.getInstance().isInShootingMode()) {
          controller.setGoal(-setting);
        }

        Logger.recordOutput("Turret/wouldBeSendingIfSending", setting);
        //double position = ((angle + 180)/360 * totalEncoderDisplacement);
        Logger.recordOutput("Turret/atCurrentAngle", ((-encoder.getDistance()/totalEncoderDisplacement)*360)-180);

        //only run if you're not getting limit switch data
        if (Robot.isReal()) {  
          // Logger.recordOutput("Turret/movingToPosition",((controller.getGoal().position + 180)/360 * totalEncoderDisplacement));
          double calculate = controller.calculate(encoder.getDistance(), controller.getGoal());
          Logger.recordOutput("Turret/voltage", calculate);
          //System.out.println("SENDING VOLTAGE " + calculate);
          motor.setVoltage(calculate);
          //System.out.println(motor.getMotorVoltage().getValueAsDouble());
        } else {
            sim.update(0.2);
            double currentPosition = sim.getEncoderPosition();
            sim.setMotorVoltage(controller.calculate(currentPosition, controller.getGoal()));
        }
      
      //Design issue w intake and turret, if the turret is rotated a certain way and you run intake they'll crash into each other
      //so Felix asked me to add this conditional hard stop to keep this from happening. Bear in mind 0.9 may have to become <= 0.1
      //depending on what angle value corresponds to turret rotated all the way to the right
      if(angle >= 0.9) { Intake.getInstance().SetVolts(0);}
      
  }

    public Double getMotorVoltage() {
      return motor.getMotorVoltage().getValueAsDouble();
    }

    public void resetEncoder() {
      encoder.reset();
      System.out.println("encoder reset");
    }
    }
