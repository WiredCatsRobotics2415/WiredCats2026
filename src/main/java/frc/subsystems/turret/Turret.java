package frc.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.shooter.Shooter;
import frc.visualization.BallSim;
import frc.constants.Measurements;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public double currentPitch = 0.0;
    private static Servo turret1 = new Servo(0);
    private static Servo turret2 = new Servo(1);
    private boolean currentlyUp = false;

    public TurretSim sim = new TurretSim();

    private static TalonFX motor;
    final PositionVoltage m_request = new PositionVoltage(0);

    private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(TurretConstants.kMaxVelocity, TurretConstants.kMaxAcceleration);
    private final ProfiledPIDController controller =
      new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, constraints, 0.02);
    
    public static Turret getInstance() {
        motor = new TalonFX(PortNumbers.Turret_Motor);

        if (instance == null) instance = new Turret(); 
        return instance; 
    }

    public Turret() {
      encoder.reset();
      encoder.setDistancePerPulse(0.001);
    }

    public void setAngle(double angle) {
      if (angle < Measurements.MaxTurretAngle && angle > Measurements.MinTurretAngle) {
            currentAngle = angle;
            double position = angle / 360.0; // Convert from degrees to rotations
            if (position > 0 && position < 1) {
              controller.setGoal(position);
            }
      }
    }

  //   public void movePosChange(double change) {
  //     System.out.println("CURRENT MOTOR POSITION:");
  //     System.out.println(motor.getPosition().getValueAsDouble());
  //     System.out.println("CURRENT GOAL POSITION:");
  //     System.out.println(motor.getPosition().getValueAsDouble() + change);
  //     motor.setControl(m_request.withPosition(motor.getPosition().getValueAsDouble() + change));
  // }

    public void setControllerChange(double posChange) {
      controller.setGoal(controller.getGoal().position + posChange);
    }

    public void setPitchAngle(double angle) {
      currentPitch = angle;
      if (angle < Measurements.MaxTurretPitchAngle && angle > Measurements.MinTurretPitchAngle) {
          turret1.setAngle(Measurements.TurretPitchOffset+angle);
          turret2.setAngle(Measurements.TurretPitchOffset+angle);
      }
    }

    public void IpswitchPitchSwitch() {
      if (currentlyUp==false) {
        turret1.setAngle(45);
        turret2.setAngle(45);
      } else {
        turret1.setAngle(15);
        turret2.setAngle(15);
      }
    }

    public double getPitchAngle() {
      return currentPitch;
    }

    public double getAngle() {
        return currentAngle;
    }

    public double relativeToTurretAngle(double angle) {
      return (angle + 90 + 360) % 360;
    }

    @Override
    public void periodic() {
      // System.out.println("CURRENTLY AT:");
      // System.out.println(motor.getPosition().getValueAsDouble());
      // System.out.println("SENDING VOLTS TO CORRECT:");
      //System.out.println(motor.getMotorVoltage());
      System.out.println("SENDING GOAL v. ACTUAL");
      System.out.println(controller.getGoal().position);
      System.out.println(encoder.getDistance());
      Logger.recordOutput("Turret/controllerGoal", controller.getGoal().position);
      Logger.recordOutput("Turret/encoderPos", encoder.getDistance());
        //get the angle from the shooter, convert to relative turret angle, and set that as the goal for the turret
        if (RobotContainer.getInstance().isInShootingMode()) {
        double angle = shooter.getLastAngle();
        double newAngle = relativeToTurretAngle(angle);
        setAngle(newAngle);
        Logger.recordOutput("Turret/AngleGoal", newAngle);
        }

        if (Robot.isReal()) {
          //based on encoder distance, set the motor to voltage necessary via PID
          //encoder.get() returns a value from 0 to 1, representing the position of the turret within its 360 degree rotation, 
          //which is what it needs for the calculate()
          double calculate = controller.calculate(encoder.getDistance(), controller.getGoal());
          System.out.println(calculate);
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
