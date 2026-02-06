// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation class for the Shooter subsystem.
 * Provides realistic flywheel physics simulation for testing PID control.
 */
public class ShooterSim {
    // Simulation parameters
    private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getFalcon500(1); // Single Falcon 500 per flywheel
    private static final double FLYWHEEL_GEARING = 1.0; // Direct drive, adjust if geared
    private static final double FLYWHEEL_MOI_KG_M2 = 0.005; // Moment of inertia in kg*m^2 (adjust based on actual flywheel)

    // Flywheel simulators
    private final FlywheelSim flywheel1Sim;
    private final FlywheelSim flywheel2Sim;

    // Current voltage being applied to motors
    private double flywheel1Voltage = 0.0;
    private double flywheel2Voltage = 0.0;

    // Indexer motor simulation (simplified)
    private double indexerVoltage = 0.0;

    // Encoder simulation
    private double encoderPosition = 0.0; // In rotations
    private double encoderVelocity = 0.0; // In rotations per second

    /**
     * Creates a new ShooterSim with realistic flywheel physics.
     */
    public ShooterSim() {
        // Create flywheel simulations using WPILib's FlywheelSim
        // This automatically handles motor dynamics and rotational inertia
        // FlywheelSim requires a LinearSystem, which we create from the DC motor model
        LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.createFlywheelSystem(
            FLYWHEEL_GEARBOX,
            FLYWHEEL_MOI_KG_M2,
            FLYWHEEL_GEARING
        );

        flywheel1Sim = new FlywheelSim(flywheelPlant, FLYWHEEL_GEARBOX);
        flywheel2Sim = new FlywheelSim(flywheelPlant, FLYWHEEL_GEARBOX);
    }

    /**
     * Updates the simulation with the current motor voltages.
     * Should be called periodically (typically every 20ms).
     *
     * @param dt Time step in seconds (typically 0.02 for 20ms loop)
     */
    public void update(double dt) {
        // Update flywheel simulations with applied voltages
        flywheel1Sim.setInputVoltage(flywheel1Voltage);
        flywheel2Sim.setInputVoltage(flywheel2Voltage);

        // Step the simulations forward
        flywheel1Sim.update(dt);
        flywheel2Sim.update(dt);

        // Simulate encoder reading as average of both flywheels
        // Convert from rad/s to rotations/s
        double flywheel1VelocityRPS = flywheel1Sim.getAngularVelocityRPM() / 60.0;
        double flywheel2VelocityRPS = flywheel2Sim.getAngularVelocityRPM() / 60.0;
        encoderVelocity = (flywheel1VelocityRPS + flywheel2VelocityRPS) / 2.0;

        // Update encoder position
        encoderPosition += encoderVelocity * dt;

        // Log simulation data
        Logger.recordOutput("ShooterSim/Flywheel1VelocityRPS", flywheel1VelocityRPS);
        Logger.recordOutput("ShooterSim/Flywheel2VelocityRPS", flywheel2VelocityRPS);
        Logger.recordOutput("ShooterSim/EncoderVelocityRPS", encoderVelocity);
        Logger.recordOutput("ShooterSim/EncoderPosition", encoderPosition);
        Logger.recordOutput("ShooterSim/Flywheel1Voltage", flywheel1Voltage);
        Logger.recordOutput("ShooterSim/Flywheel2Voltage", flywheel2Voltage);
        Logger.recordOutput("ShooterSim/Flywheel1CurrentDraw", flywheel1Sim.getCurrentDrawAmps());
        Logger.recordOutput("ShooterSim/Flywheel2CurrentDraw", flywheel2Sim.getCurrentDrawAmps());
        Logger.recordOutput("ShooterSim/IndexerVoltage", indexerVoltage);
    }

    /**
     * Sets the voltage for flywheel 1.
     * Simulates setting the motor controller output.
     *
     * @param voltage Voltage to apply (-12 to 12 volts)
     */
    public void setFlywheel1Voltage(double voltage) {
        this.flywheel1Voltage = clampVoltage(voltage);
    }

    /**
     * Sets the voltage for flywheel 2.
     * Simulates setting the motor controller output.
     *
     * @param voltage Voltage to apply (-12 to 12 volts)
     */
    public void setFlywheel2Voltage(double voltage) {
        this.flywheel2Voltage = clampVoltage(voltage);
    }

    /**
     * Sets the output for the indexer motor.
     *
     * @param output Motor output (-1.0 to 1.0)
     */
    public void setIndexerOutput(double output) {
        this.indexerVoltage = output * 12.0; // Convert to voltage
    }

    /**
     * Gets the simulated velocity of flywheel 1 in rotations per second.
     *
     * @return Velocity in RPS
     */
    public double getFlywheel1VelocityRPS() {
        return flywheel1Sim.getAngularVelocityRPM() / 60.0;
    }

    /**
     * Gets the simulated velocity of flywheel 2 in rotations per second.
     *
     * @return Velocity in RPS
     */
    public double getFlywheel2VelocityRPS() {
        return flywheel2Sim.getAngularVelocityRPM() / 60.0;
    }

    /**
     * Gets the average velocity of both flywheels in rotations per second.
     * Simulates the getSpeed() method from the real Shooter class.
     *
     * @return Average velocity in RPS
     */
    public double getAverageVelocityRPS() {
        return (getFlywheel1VelocityRPS() + getFlywheel2VelocityRPS()) / 2.0;
    }

    /**
     * Gets the simulated encoder velocity in rotations per second.
     *
     * @return Encoder velocity in RPS
     */
    public double getEncoderRate() {
        return encoderVelocity;
    }

    /**
     * Gets the simulated encoder position in rotations.
     *
     * @return Encoder position in rotations
     */
    public double getEncoderPosition() {
        return encoderPosition;
    }

    /**
     * Resets the encoder position to zero.
     */
    public void resetEncoder() {
        encoderPosition = 0.0;
    }

    /**
     * Gets the current draw of flywheel 1.
     *
     * @return Current in amps
     */
    public double getFlywheel1Voltage() {
        return flywheel1Sim.getInputVoltage();
    }

    /**
     * Gets the current draw of flywheel 2.
     *
     * @return Current in amps
     */
    public double getFlywheel2Voltage() {
        return flywheel2Sim.getInputVoltage();
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

    /**
     * Gets encoder rate with simulated noise for more realistic testing.
     *
     * @return Noisy encoder rate in RPS
     */
    public double getEncoderRateWithNoise() {
        return addNoise(encoderVelocity, 0.1); // Add ±0.1 RPS noise
    }
}
