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
    public double currentPosition = 0.5;
    public Servo turret1;
    public Servo turret2;
    private boolean currentlyUp = false;

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
      turret1 = new Servo(5);;
      turret2 = new Servo(6);
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

    public void setControllerChange(double posChange) {
      controller.setGoal(controller.getGoal().position + posChange);
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
        turret1.set(0.48);
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

        //get the angle from the shooter, convert to relative turret angle, then 
        //convert that to a 0-1 position, and set that as the controller goal for the turret
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
