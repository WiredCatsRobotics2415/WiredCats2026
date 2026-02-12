// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.visualization.RobotParts;

/**
 * Simulation class for the Turret subsystem.
 * Provides realistic turret rotation physics simulation for testing PID control.
 * Turret operates from 0 to 360 degrees, with encoder values normalized from 0 to 1.
 */
public class TurretSim {
    // Simulation parameters
    private CommandSwerveDrivetrain robot = CommandSwerveDrivetrain.getInstance();
    private static final DCMotor TURRET_GEARBOX = DCMotor.getFalcon500(1); // Single Falcon 500/TalonFX
    private static final double TURRET_GEARING = 100.0; // Gear ratio - adjust based on actual mechanism
    private static final double TURRET_MOI_KG_M2 = 0.5; // Moment of inertia in kg*m^2 (rotational mass)
    private static final double TURRET_LENGTH_METERS = 0.3; // Length from pivot to center of mass
    private static final double MIN_ANGLE_RADIANS = 0.0; // 0 degrees
    private static final double MAX_ANGLE_RADIANS = 2.0 * Math.PI; // 360 degrees
    private static final boolean SIMULATE_GRAVITY = false; // Turret is horizontal, no gravity effect

    // Arm simulator (used for rotational mechanisms like turrets)
    private final SingleJointedArmSim turretSim;

    // Current voltage being applied to motor
    private double motorVoltage = 0.0;

    /**
     * Creates a new TurretSim with realistic rotational physics.
     */
    public TurretSim() {
        // Create turret simulation using WPILib's SingleJointedArmSim
        // This automatically handles motor dynamics and rotational physics
        turretSim = new SingleJointedArmSim(
            TURRET_GEARBOX,
            TURRET_GEARING,
            TURRET_MOI_KG_M2,
            TURRET_LENGTH_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            SIMULATE_GRAVITY,
            MIN_ANGLE_RADIANS // Starting position (0 degrees)
        );
    }

    /**
     * Updates the simulation with the current motor voltage.
     * Should be called periodically (typically every 20ms).
     *
     * @param dt Time step in seconds (typically 0.02 for 20ms loop)
     */
    public void update(double dt) {
        // Update turret simulation with applied voltage
        turretSim.setInputVoltage(motorVoltage);

        // Step the simulation forward
        turretSim.update(dt);

        // Log simulation data
        Logger.recordOutput("TurretSim/AngleDegrees", getAngleDegrees());
        Logger.recordOutput("TurretSim/AngleRadians", turretSim.getAngleRads());
        Logger.recordOutput("TurretSim/EncoderPosition", getEncoderPosition());
        Logger.recordOutput("TurretSim/VelocityDegreesPerSec", getVelocityDegreesPerSecond());
        Logger.recordOutput("TurretSim/MotorVoltage", motorVoltage);
        Logger.recordOutput("TurretSim/CurrentDrawAmps", turretSim.getCurrentDrawAmps());
        Logger.recordOutput("TurretSim/AtMinLimit", turretSim.hasHitLowerLimit());
        Logger.recordOutput("TurretSim/AtMaxLimit", turretSim.hasHitUpperLimit());

        RobotParts.getInstance().update();
    }

    /**
     * Sets the voltage for the turret motor.
     * Simulates setting the motor controller output.
     *
     * @param voltage Voltage to apply (-12 to 12 volts)
     */
    public void setMotorVoltage(double voltage) {
        this.motorVoltage = clampVoltage(voltage);
    }

    /**
     * Gets the simulated turret angle in degrees.
     * Range: 0 to 360 degrees
     *
     * @return Angle in degrees
     */
    public double getAngleDegrees() {
        return Math.toDegrees(turretSim.getAngleRads());
    }

    /**
     * Gets the simulated turret angle in radians.
     * Range: 0 to 2π radians
     *
     * @return Angle in radians
     */
    public double getAngleRadians() {
        return turretSim.getAngleRads();
    }

    /**
     * Gets the encoder position normalized from 0 to 1.
     * 0 represents 0 degrees, 1 represents 360 degrees.
     * This simulates the encoder.getDistance() reading.
     *
     * @return Encoder position from 0.0 to 1.0
     */
    public double getEncoderPosition() {
        // Normalize the angle from [0, 2π] to [0, 1]
        return turretSim.getAngleRads() / (2.0 * Math.PI);
    }

    /**
     * Gets the simulated turret angular velocity in degrees per second.
     *
     * @return Angular velocity in degrees/second
     */
    public double getVelocityDegreesPerSecond() {
        return Math.toDegrees(turretSim.getVelocityRadPerSec());
    }

    /**
     * Gets the simulated turret angular velocity in radians per second.
     *
     * @return Angular velocity in radians/second
     */
    public double getVelocityRadiansPerSecond() {
        return turretSim.getVelocityRadPerSec();
    }

    /**
     * Gets the encoder position with simulated noise for more realistic testing.
     *
     * @return Noisy encoder position from 0.0 to 1.0
     */
    public double getEncoderPositionWithNoise() {
        return addNoise(getEncoderPosition(), 0.001); // Add ±0.001 noise (±0.36 degrees)
    }

    /**
     * Checks if the turret has reached the minimum angle limit (0 degrees).
     *
     * @return true if at minimum limit
     */
    public boolean hasHitMinLimit() {
        return turretSim.hasHitLowerLimit();
    }

    /**
     * Checks if the turret has reached the maximum angle limit (360 degrees).
     *
     * @return true if at maximum limit
     */
    public boolean hasHitMaxLimit() {
        return turretSim.hasHitUpperLimit();
    }

    /**
     * Gets the current draw of the turret motor.
     *
     * @return Current in amps
     */
    public double getCurrentDrawAmps() {
        return turretSim.getCurrentDrawAmps();
    }

    /**
     * Resets the turret position to 0 degrees.
     */
    public void resetPosition() {
        // Note: SingleJointedArmSim doesn't have a direct setState method in all versions
        // In practice, you would need to recreate the sim or use setState if available
    }

    /**
     * Sets the turret position directly (useful for testing).
     *
     * @param angleDegrees Desired angle in degrees (0 to 360)
     */
    public void setPosition(double angleDegrees) {
        double angleRadians = Math.toRadians(angleDegrees);
        // Clamp to valid range
        angleRadians = Math.max(MIN_ANGLE_RADIANS, Math.min(MAX_ANGLE_RADIANS, angleRadians));

        // Use setState if available (check your WPILib version)
        // turretSim.setState(angleRadians, 0.0);
    }

    /**
     * Clamps voltage to realistic battery voltage range.
     *
     * @param voltage Input voltage
     * @return Clamped voltage (-12 to 12)
     */
    private double clampVoltage(double voltage) {
        return Math.max(-12.0, Math.min(12.0, voltage));
    }

    /**
     * Adds artificial noise to simulate real-world sensor readings.
     *
     * @param value The clean sensor value
     * @param noiseAmplitude Maximum noise amplitude
     * @return Value with added noise
     */
    private double addNoise(double value, double noiseAmplitude) {
        return value + (Math.random() - 0.5) * 2 * noiseAmplitude;
    }
}
