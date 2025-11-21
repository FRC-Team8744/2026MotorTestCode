// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.motorKraken;
import frc.robot.subsystems.motorSparkMax;
import frc.robot.subsystems.sensorAbsEnc;

public class TurnMotor extends SubsystemBase {
   private final motorSparkMax m_TurnMotorSparkMax;
   private final sensorAbsEnc m_canCoder;
  /** Creates a new TurnMotor. */
  public TurnMotor() {
    m_TurnMotorSparkMax = new motorSparkMax(10);
    m_canCoder = new sensorAbsEnc(9, 247.23);
  }
  public void set(double speed) {
    m_TurnMotorSparkMax.set_dutycycle(speed);
  }

  public void setVelocity(double velocity) {
    m_TurnMotorSparkMax.set_velocity(velocity);
  }

  public void setPosition(double position) {
    m_TurnMotorSparkMax.set_position(position);
  }

  public void stop() {
    m_TurnMotorSparkMax.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("Rotator postion", m_TurnMotorSparkMax.getPosition());
     SmartDashboard.putNumber("Absolute encoder position", m_canCoder.getCanCoder());
  }
  }

