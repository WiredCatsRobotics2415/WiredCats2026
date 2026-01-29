// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Subsystems Constants, TODO: currently all placeholders change once we have real values
import Subsystems.FLYWHEEL_1_ID;
import Subsystems.FLYWHEEL_2_ID;
import Subsystems.INTAKE_MOTOR_ID;
import Subsystems.INTAKE_SPEED;

public class ShooterSubsystem {

  private static ShooterSubsystem instance = null;
  private static Talonx60 flywheel1;
  private static Talonx60 flywheel2;
  private static Talonx60 intakeMotor;

  // Intake Functions Below
  private ShooterSubsystem() {
    // Constructor, idk what to put here rn
    flywheel1 = new Talonx60(FLYWHEEL_1_ID);
    flywheel2 = new Talonx60(FLYWHEEL_2_ID);
    intakeMotor = new Talonx44(INTAKE_MOTOR_ID);

  }

  public static ShooterSubsystem getInstance() {
    if (instance == null)
      instance = new ShooterSubsystem();
    return instance;
  }

  public void startShooting() // Starts Intake
  {
    IntakeMotor.setsetControl(new VelocityDutyCycle(INTAKE_SPEED)); // Currently working on RPS output, can change to
                                                                    // RPM if needed
  }

  public void stopShooting() // Stops Intake
  {
    IntakeMotor.set(0.0);
  }

  // Flywheel Functions Below
  public float getSpeed() // Returns average speed of both flywheels
  {
    return (Flywheel1.speed() + Flywheel2.speed()) / 2;
  }

  public void speedUp(float speed) // Sets Flywheel speed, ask if Override with set SpeedUp paramater is needed
  {
    Flywheel1.set(speed);// Currently working on percentage output, ask if it should be defined in RPM or
                         // smth
    Flywheel2.set(speed);
  }

  public void speedDown() // Stops Flywheel
  {
    Flywheel1.set(0.0);
    Flywheel2.set(0.0);
  }

}
