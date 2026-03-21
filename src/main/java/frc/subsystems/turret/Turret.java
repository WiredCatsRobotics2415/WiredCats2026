package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.shooter.Shooter;
import frc.visualization.BallSim;
import frc.constants.Measurements;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import org.apache.commons.math3.util.MathArrays.Position;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final Encoder encoder = new Encoder(8, 7);
    public double currentAngle = 0.0;
    public double currentPosition = 0.5;
    public Servo turret1;
    public Servo turret2;
    private boolean currentlyUp = false;

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
      turret1 = new Servo(PortNumbers.TurretServo1);;
      turret2 = new Servo(PortNumbers.TurretServo2);

      leftLimitSwitch = new DigitalInput(PortNumbers.TurretLimit1);
      rightLimitSwitch = new DigitalInput(PortNumbers.TurretLimit2);

      encoder.reset();
      encoder.setDistancePerPulse(0.001);
    }

    public void setAngle(double angle) {
      if (angle < Measurements.MaxTurretAngle && angle > Measurements.MinTurretAngle) {
            currentAngle = angle;
            double position = angle / 60; // Convert from degrees to rotations
            if (position > -3 && position < 3) {
              Logger.recordOutput("Turret/SettingPosition", position);
              controller.setGoal(position);
            }
      }
    }

    public void setControllerChange(double posChange) {
      controller.setGoal(controller.getGoal().position + posChange);
      System.out.println(controller.getGoal().position + posChange);
    }

    public void IpswitchPitchSwitch() {
      if (currentlyUp==false) {
      //up is 0.88 for turret1, down is 0.48
      //down is 0.95 for turret2, up is 0.55
      System.out.println("GOING UP");
        turret1.set(0.88);
        turret2.set(0.55);
        currentlyUp = true;
      } else {
        System.out.println("GOING DOWN");
        turret1.set(0.3);
        turret2.set(0.95);
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
      Logger.recordOutput("Turret/encoderPos", encoder.getDistance());

      Logger.recordOutput("Turret/turret1", turret1.get());
      Logger.recordOutput("Turret/turret2", turret2.get());

      Logger.recordOutput("Turret/leftlimit", leftLimitSwitch.get());
      Logger.recordOutput("Turret/rightlimit", rightLimitSwitch.get());

      turret1.set(turret1.get());
      turret2.set(turret2.get());

      if (leftLimitSwitch.get()) {
        encoder.reset();
      }
    
        //get the angle from the shooter, convert to relative turret angle, then 
        //convert that to a 0-1 position, and set that as the controller goal for the turret
        double angle = shooter.getLastAngle();
        double newAngle = relativeToTurretAngle(Math.toDegrees(angle));
        Logger.recordOutput("Turret/AngleGoal", newAngle);

        if (RobotContainer.getInstance().isInShootingMode()) {
          setAngle(newAngle);
        }

        //only run if you're not getting limit switch data
        if (Robot.isReal()) {  
          double calculate = controller.calculate(encoder.getDistance(), controller.getGoal());
          Logger.recordOutput("Turret/voltage", calculate);
          if ((leftLimitSwitch.get()==true) && (calculate>0)) {
            System.out.println("hitting left switch and going left");
            calculate = 0;
          } else if ((rightLimitSwitch.get()==true) && (calculate<0)) {
            System.out.println("hitting right switch and going right");
            calculate = 0;
          }
          motor.setVoltage(-calculate);
        } else {
            sim.update(0.2);
            double currentPosition = sim.getEncoderPosition();
            sim.setMotorVoltage(controller.calculate(currentPosition, controller.getGoal()));
        }
  }

    public Double getMotorVoltage() {
      return motor.getMotorVoltage().getValueAsDouble();
    }
    }
