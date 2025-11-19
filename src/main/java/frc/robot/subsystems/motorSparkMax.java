// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/*
 * Documentation links:
 * https://docs.revrobotics.com/brushless/neo/v1.1
 * https://github.com/REVrobotics/REVLib-Examples/tree/main/Java/SPARK/Open%20Loop%20Arcade%20Drive
 */

public class motorSparkMax {
  private final SparkMax m_motor;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private SparkMaxConfig config_motor;

  /**
   * motorSparkMax - A class to simplify Rev Robotics Neo 1.1 motor drivers
   *
   * @param motorCAN_ID The CAN ID of the motor.
   */
  public motorSparkMax(int motorCAN_ID) {
    // Create motor object
    m_motor = new SparkMax(motorCAN_ID, MotorType.kBrushless);

    // Create other motor objects
    closedLoopController = m_motor.getClosedLoopController();
    encoder = m_motor.getEncoder();
  
  
    // Create a motor object default configuration
    config_motor = new SparkMaxConfig();

    // User can optionally change the configs or leave it alone to perform a factory default
    config_motor.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    config_motor.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    config_motor.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    // Set motor options that were not included in the default configuration set
    config_motor
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the updated configuration to the motor
    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_motor.configure(config_motor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(0);  // Zero the motor encoder position
  }

  // Set motor speed with a fractional value from -1.0 to 1.0
  public void set_dutycycle(double dutycycle) {
    // Update the motor with the new value
    m_motor.set(dutycycle);
  }

  // Set the motor's goal to an exact velocity in rotations per second
  public void set_velocity(double velocity_goal) {
    // Update the motor with the new control value
    closedLoopController.setReference(velocity_goal, ControlType.kVelocity, ClosedLoopSlot.kSlot1); 
    // closedLoopController.setReference(velocity_goal, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
  }

  // Set the motor's goal to an exact position
  public void set_position(double position_goal) {
    // Update the motor with the new control value
    closedLoopController.setReference(position_goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);  
  }

  // Reset the motor position encoder to zero
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  // Stop the motor
  public void stop() {
    m_motor.stopMotor();
  }

  // Returns the motor encoder position
  public double getPosition() {
    return encoder.getPosition();
  }

  // Returns true if the motor is at the setpoint position (within 1 unit)
  public boolean isAtSetpoint() {
    return false; //Math.abs(m_motor.getClosedLoopError().getValueAsDouble()) <= 1.0;
  }
}
