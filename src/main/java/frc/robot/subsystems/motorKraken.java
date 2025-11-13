// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/*
 * Documentation links:
 * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/index.html
 * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/examples/quickstart.html
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/ArcadeDrive/src/main/java/frc/robot/Robot.java
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java
 */

public class motorKraken {
  private final TalonFX m_motor;

  // Control objects
  private DutyCycleOut ctrl_DutyCycle = new DutyCycleOut(0);
  private VelocityVoltage ctrl_Velocity = new VelocityVoltage(0);  // Uses Slot0
  private PositionVoltage ctrl_Position = new PositionVoltage(0);  // Uses Slot1
  private NeutralOut m_brake = new NeutralOut();  // Create a neutral control so we can disable the motor

  private TalonFXConfiguration config_motor = new TalonFXConfiguration();

  /**
   * motorKraken - A class to simplify Cross The Road Electronics "Kraken" motor drivers
   *
   * @param motorCAN_ID The CAN ID of the motor.
   */
  public motorKraken(int motorCAN_ID) {
    // Create motor object
    m_motor = new TalonFX(motorCAN_ID);

    // Create a motor object default configuration
    config_motor = new TalonFXConfiguration();

    // User can optionally change the configs or leave it alone to perform a factory default
    // Set slot gain values for velocity closed-loop control
    config_motor.Slot0.kS = 0.1; // Static Feedforward Gain: To account for friction, add static feedforward value (in Volts)
    config_motor.Slot0.kV = 0.12; // Velocity Feedforward Gain: Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    config_motor.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    config_motor.Slot0.kP = 0.11; // Proportional Gain
    config_motor.Slot0.kI = 0; // Integral Gain
    config_motor.Slot0.kD = 0; // Derivative Gain
    // Peak output of 8 volts
    config_motor.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    // Set slot gain values for position closed-loop control
    config_motor.Slot1.kS = 0; // Static Feedforward Gain: To account for friction, add static feedforward value (in Volts)
    config_motor.Slot1.kP = 2.4; // Proportional Gain
    config_motor.Slot1.kI = 0; // Integral Gain
    config_motor.Slot1.kD = 0.1; // Derivative Gain

    // Configure the gear ratio between the motor and mechanism
    // config_motor.Feedback.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    // Configure velocity and acceleration limiting
    // config_motor.MotionMagic
    //   // 5 (mechanism) rotations per second cruise
    //   .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
    //   // Take approximately 0.5 seconds to reach max vel
    //   .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
    //   // Take approximately 0.1 seconds to reach max accel
    //   .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    // Apply the updated configuration to the motor
    m_motor.getConfigurator().apply(config_motor);

    // Set motor options that were not included in the configuration set
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    m_motor.setPosition(0);  // Zero the motor encoder position

    // Turn off CTRE's FOC (Field Oriented Control) algorithm (requires a "PRO" license)
    ctrl_DutyCycle.EnableFOC = false;
    ctrl_Position.EnableFOC = false;
    ctrl_Velocity.EnableFOC = false;

    // Set the memory slot number for each control object that uses PID gains
    ctrl_Velocity.Slot = 0;
    ctrl_Position.Slot = 1;
  }

  // Set motor speed with a fractional value from -1.0 to 1.0
  public void set_dutycycle(double dutycycle) {
    // Set the value of the controller's output
    ctrl_DutyCycle.Output = dutycycle;
    // Update the motor with the new control value
    m_motor.setControl(ctrl_DutyCycle);
  }

  // Set the motor's goal to an exact velocity in rotations per second
  public void set_velocity(double velocity_goal) {
    // Set the goal value of the PID controller
    ctrl_Velocity.Velocity = velocity_goal;
    // Update the motor with the new control value
    m_motor.setControl(ctrl_Velocity);
  }

  // Set the motor's goal to an exact position
  public void set_position(double position_goal) {
    // Set the goal value of the PID controller
    ctrl_Position.Position = position_goal;
    // Update the motor with the new control value
    m_motor.setControl(ctrl_Position);
  }

  // Reset the motor position encoder to zero
  public void resetEncoder() {
    m_motor.setPosition(0);
  }

  // Stop the motor
  public void stop() {
    m_motor.setControl(m_brake);
    m_motor.stopMotor();
  }

  // Returns the motor encoder position
  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  // Returns true if the motor is at the setpoint position (within 1 unit)
  public boolean isAtSetpoint() {
    return Math.abs(m_motor.getClosedLoopError().getValueAsDouble()) <= 1.0;
  }
}
