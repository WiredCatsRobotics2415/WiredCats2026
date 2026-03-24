package frc.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ClimberConstants;
import frc.constants.Subsystems.IntakeConstants;
import frc.constants.Subsystems.ShooterConstants;
import frc.constants.Subsystems.TurretConstants;
import frc.constants.Subsystems.PortNumbers;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    private static Intake instance = null;
    public TalonFX intakePush;
    private TalonFX intakeDrive;
    public boolean isOut = false;
    public PositionVoltage m_request = new PositionVoltage(0);
    private DigitalInput frontLimit = new DigitalInput(PortNumbers.Intake_Front_Limit_ID); 
    private DigitalInput backLimit = new DigitalInput(PortNumbers.Intake_Back_Limit_ID); 
    private double amountToMove = IntakeConstants.goalDistance;

      // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity.get(), IntakeConstants.kMaxAcceleration.get());
    private final ProfiledPIDController controller =
      new ProfiledPIDController(IntakeConstants.kP.get(), IntakeConstants.kI.get(), IntakeConstants.kD.get(), constraints, 0.02);
    
    public static Intake getInstance() {
    if (instance == null)
      instance = new Intake();
    return instance;
  }

  private Intake() {
    intakePush = new TalonFX(PortNumbers.Intake_Motor_ID);
    intakeDrive = new TalonFX(PortNumbers.Intake_Drive_ID);
    intakePush.setNeutralMode(NeutralModeValue.Coast);
    intakePush.setPosition(0);
    //TODO: define Intake Drive Motor
  }

  public void switchSpinForComp() {
    System.out.println("Switch Spin For Comp");
    if (isOut) {
      controller.setGoal(0);
      isOut = false;
    } else {
      controller.setGoal(amountToMove);
      isOut = true;
    }
  }

  public BooleanSupplier getFrontSwitchSupplier() {
    return () -> frontLimit.get();
  }

  public BooleanSupplier getBackSwitchSupplier() {
    return () -> backLimit.get();
  }

    public Command SetVolts(double voltage) {
        if ((backLimit.get()) && (frontLimit.get())) {
            return runOnce(
        () -> {
          intakePush.setVoltage(voltage);
        });
        } else {
            return runOnce(
        () -> {
          intakePush.setVoltage(0);
        });
        }
    }

    @Override
      public void periodic() {
      //if not there
      if(frontLimit.get() == false){System.out.println("LIMIT SWITCH RETURNS FALSE!!!");}
      if (controller.atGoal() || !frontLimit.get()) { 
        //System.out.println("Not Moving");  
        intakePush.setVoltage(0);
        if (backLimit.get()) {
          intakeDrive.setVoltage(0);
        }

      } else {

        //System.out.println("Moving");
        double calculate = controller.calculate(-intakePush.getPosition().getValueAsDouble(), controller.getGoal());
        intakePush.setVoltage(-calculate*2);
      }

      Logger.recordOutput("Intake/currentPos", -intakePush.getPosition().getValueAsDouble());
      Logger.recordOutput("Intake/controllerGoal", controller.getGoal().position);
      Logger.recordOutput("Intake/voltage", intakePush.getMotorVoltage().getValueAsDouble());
      //System.out.println("Intake/currentPos"+ -intakePush.getPosition().getValueAsDouble());
      //System.out.println("Intake/controllerGoal"+ controller.getGoal().position);
      //System.out.println("Intake/voltage"+intakePush.getMotorVoltage().getValueAsDouble());
    }

    public void Switch(double voltage) {
        if (backLimit.get()) {
            //move forward until we hit the limit switch
            SetVolts(voltage).until(getFrontSwitchSupplier());
        } else if (frontLimit.get()) {
            //move backward until we hit the limit switch
            SetVolts(-voltage).until(getBackSwitchSupplier());
        }
    }
}
