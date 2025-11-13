// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.motorKraken;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final motorKraken m_Elevator;

  public Elevator() {
    m_Elevator = new motorKraken(1);  //Put magic number in Constants!
  }

  public void set(double speed) {
    m_Elevator.set_dutycycle(speed);
  }

  public void setVelocity(double velocity) {
    m_Elevator.set_velocity(velocity);
  }

  public void setPosition(double position) {
    m_Elevator.set_position(position);
  }

  public void stop() {
    m_Elevator.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", m_Elevator.getPosition());
  }
}
