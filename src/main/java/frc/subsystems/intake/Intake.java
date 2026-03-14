package frc.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.constants.Subsystems.PortNumbers;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    private static Intake instance = null;
    public TalonFX intakePush;
    private TalonFX intakeSpin;
    public boolean isOut = false;
    public PositionVoltage m_request = new PositionVoltage(0);
    private DigitalInput frontLimit = new DigitalInput(PortNumbers.Intake_Front_Limit_ID); 
    private DigitalInput backLimit = new DigitalInput(PortNumbers.Intake_Back_Limit_ID); 

      // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
    private final PIDController pid = new PIDController(ClimberConstants.kP, 0.0, 0.0);
    
    public static Intake getInstance() {
    if (instance == null)
      instance = new Intake();
    return instance;
  }

  private Intake() {
    intakePush = new TalonFX(PortNumbers.Intake_Motor_ID);
    //TODO: define Intake Drive Motor
  }

  public void switchSpinForComp() {
    if (isOut) {
      intakePush.setControl(m_request.withPosition(intakePush.getPosition().getValueAsDouble()+1));
      isOut = false;
    } else {
      intakePush.setControl(m_request.withPosition(intakePush.getPosition().getValueAsDouble()-1));
      isOut = true;
    }
  }

  public Command setSpin(double speed) {
    return runOnce(
        () -> {
          intakeSpin.set(speed);
        });
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

    public void Switch(double voltage) {
        if (backLimit.get()) {
            //move forward until we hit the limit switch
            SetVolts(0.3).until(getFrontSwitchSupplier());
        } else if (frontLimit.get()) {
            //move backward until we hit the limit switch
            SetVolts(-0.3).until(getBackSwitchSupplier());
        }
    }
}
