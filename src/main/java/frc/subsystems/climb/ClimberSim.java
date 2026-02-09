// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulation class for the Climber subsystem.
 * Provides realistic elevator/climber physics simulation for testing PID and feedforward control.
 */
public class ClimberSim {
    // Simulation parameters
    private static final DCMotor CLIMBER_GEARBOX = DCMotor.getFalcon500(1); // Single Falcon 500/TalonFX
    private static final double CLIMBER_GEARING = 10.0; // Gear ratio - adjust based on actual mechanism
    private static final double CLIMBER_CARRIAGE_MASS_KG = 5.0; // Mass of climbing mechanism in kg
    private static final double CLIMBER_DRUM_RADIUS_METERS = 0.02; // Radius of winch drum in meters (adjust as needed)
    private static final double MIN_HEIGHT_METERS = 0.0; // Minimum climb height
    private static final double MAX_HEIGHT_METERS = 1.5; // Maximum climb height (adjust as needed)
    private static final boolean SIMULATE_GRAVITY = true; // Whether to simulate gravity effects

    // Elevator simulator
    private final ElevatorSim elevatorSim;

    // Current voltage being applied to motor
    private double motorVoltage = 0.0;

    // Wire potentiometer simulation (outputs voltage proportional to position)
    private static final double POTENTIOMETER_MIN_VOLTAGE = 0.0; // Voltage at minimum extension
    private static final double POTENTIOMETER_MAX_VOLTAGE = 5.0; // Voltage at maximum extension

    /**
     * Creates a new ClimberSim with realistic elevator physics.
     */
    public ClimberSim() {
        // Create elevator simulation using WPILib's ElevatorSim
        // This automatically handles motor dynamics, gravity, and position limits
        elevatorSim = new ElevatorSim(
            CLIMBER_GEARBOX,
            CLIMBER_GEARING,
            CLIMBER_CARRIAGE_MASS_KG,
            CLIMBER_DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            SIMULATE_GRAVITY,
            MIN_HEIGHT_METERS // Starting position
        );
    }

    /**
     * Updates the simulation with the current motor voltage.
     * Should be called periodically (typically every 20ms).
     *
     * @param dt Time step in seconds (typically 0.02 for 20ms loop)
     */
    public void update(double dt) {
        // Update elevator simulation with applied voltage
        elevatorSim.setInputVoltage(motorVoltage);

        // Step the simulation forward
        elevatorSim.update(dt);

        // Log simulation data
        Logger.recordOutput("ClimberSim/PositionMeters", elevatorSim.getPositionMeters());
        Logger.recordOutput("ClimberSim/VelocityMetersPerSec", elevatorSim.getVelocityMetersPerSecond());
        Logger.recordOutput("ClimberSim/MotorVoltage", motorVoltage);
        Logger.recordOutput("ClimberSim/CurrentDrawAmps", elevatorSim.getCurrentDrawAmps());
        Logger.recordOutput("ClimberSim/PotentiometerVoltage", getPotentiometerVoltage());
        Logger.recordOutput("ClimberSim/AtLowerLimit", elevatorSim.hasHitLowerLimit());
        Logger.recordOutput("ClimberSim/AtUpperLimit", elevatorSim.hasHitUpperLimit());
    }

    /**
     * Sets the voltage for the climb motor.
     * Simulates setting the motor controller output.
     *
     * @param voltage Voltage to apply (-12 to 12 volts)
     */
    public void setMotorVoltage(double voltage) {
        this.motorVoltage = clampVoltage(voltage);
    }

    /**
     * Gets the simulated position of the climber in meters.
     *
     * @return Position in meters
     */
    public double getPositionMeters() {
        return elevatorSim.getPositionMeters();
    }

    /**
     * Gets the simulated velocity of the climber in meters per second.
     *
     * @return Velocity in m/s
     */
    public double getVelocityMetersPerSecond() {
        return elevatorSim.getVelocityMetersPerSecond();
    }

    /**
     * Gets the simulated wire potentiometer voltage.
     * The potentiometer voltage is proportional to the climber position.
     * This simulates the AnalogInput reading from the real climber.
     *
     * @return Voltage from 0V to 5V based on position
     */
    public double getPotentiometerVoltage() {
        // Linear mapping from position to voltage
        double positionRatio = (elevatorSim.getPositionMeters() - MIN_HEIGHT_METERS)
                             / (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS);
        return POTENTIOMETER_MIN_VOLTAGE + (positionRatio * (POTENTIOMETER_MAX_VOLTAGE - POTENTIOMETER_MIN_VOLTAGE));
    }

    /**
     * Gets the wire potentiometer voltage with simulated noise for more realistic testing.
     *
     * @return Noisy potentiometer voltage
     */
    public double getPotentiometerVoltageWithNoise() {
        return addNoise(getPotentiometerVoltage(), 0.01); // Add ±0.01V noise
    }

    /**
     * Checks if the climber has reached the lower limit.
     *
     * @return true if at lower limit
     */
    public boolean hasHitLowerLimit() {
        return elevatorSim.hasHitLowerLimit();
    }

    /**
     * Checks if the climber has reached the upper limit.
     *
     * @return true if at upper limit
     */
    public boolean hasHitUpperLimit() {
        return elevatorSim.hasHitUpperLimit();
    }

    /**
     * Gets the current draw of the climb motor.
     *
     * @return Current in amps
     */
    public double getCurrentDrawAmps() {
        return elevatorSim.getCurrentDrawAmps();
    }

    /**
     * Resets the climber position to the minimum height.
     */
    public void resetPosition() {
        // Note: ElevatorSim doesn't have a direct reset method, but we can create a new instance
        // or just track that position should be reset. For now, this is a placeholder.
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
