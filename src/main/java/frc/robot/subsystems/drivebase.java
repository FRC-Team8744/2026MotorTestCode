// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.motorKraken;
import frc.robot.subsystems.motorSparkMax;
import frc.robot.subsystems.sensorAbsEnc;

public class drivebase extends SubsystemBase {
  private final motorKraken m_leftFrontDrive;
  private final motorKraken m_leftRearDrive;
  private final motorKraken m_RightFrontDrive;
  private final motorKraken m_RightRearDrive;

  private final motorSparkMax m_leftFrontTurn;
  private final motorSparkMax m_leftRearTurn;
  private final motorSparkMax m_rightFrontTurn;
  private final motorSparkMax m_rightRearTurn;

  private final sensorAbsEnc m_leftFrontEnc;
  private final sensorAbsEnc m_leftRearEnc;
  private final sensorAbsEnc m_RightFrontEnc;
  private final sensorAbsEnc m_RightRearEnc;  /** Creates a new drivebase. */
  public drivebase() {
    m_leftFrontDrive = new motorKraken(Constants.SwerveConstants.kFrontLeftDriveMotorPort);
    m_leftRearDrive = new motorKraken(Constants.SwerveConstants.kRearLeftDriveMotorPort);
    m_RightFrontDrive = new motorKraken(Constants.SwerveConstants.kFrontRightDriveMotorPort);
    m_RightRearDrive = new motorKraken(Constants.SwerveConstants.kRearRightDriveMotorPort);

    m_leftFrontTurn = new motorSparkMax(Constants.SwerveConstants.kFrontLeftTurningMotorPort);
    m_leftRearTurn = new motorSparkMax(Constants.SwerveConstants.kRearLeftTurningMotorPort);
    m_rightFrontTurn = new motorSparkMax(Constants.SwerveConstants.kFrontRightTurningMotorPort);
    m_rightRearTurn = new motorSparkMax(Constants.SwerveConstants.kRearRightTurningMotorPort);

    m_leftFrontEnc = new sensorAbsEnc(Constants.SwerveConstants.kFrontLeftMagEncoderPort, 247.23);
    m_leftRearEnc = new sensorAbsEnc(Constants.SwerveConstants.kRearLeftMagEncoderPort, 0);
    m_RightFrontEnc = new sensorAbsEnc(Constants.SwerveConstants.kFrontRightMagEncoderPort, 0);
    m_RightRearEnc = new sensorAbsEnc(Constants.SwerveConstants.kRearRightMagEncoderPort, 0);


  
    //m_Elevator = new motorKraken(8);  //Put magic number in Constants!
    // m_TurnMotorSparkMax = new motorSparkMax(10);
    // m_canCoder = new sensorAbsEnc(9, 247.23);
    
    
  }
  // public static final int kFrontLeftDriveMotorPort = 8; // 8
  // public static final int kFrontRightDriveMotorPort = 3; // 3
  // public static final int kRearLeftDriveMotorPort = 17; // 17
  // public static final int kRearRightDriveMotorPort = 20; // 20

  // public static final int kFrontLeftTurningMotorPort = 10; // 10
  // public static final int kFrontRightTurningMotorPort = 5; // 5
  // public static final int kRearLeftTurningMotorPort = 19; // 19
  // public static final int kRearRightTurningMotorPort = 22; // 22

  // public static final int kFrontLeftMagEncoderPort = 9; // 9
  // public static final int kFrontRightMagEncoderPort = 4; // 4
  // public static final int kRearLeftMagEncoderPort = 18; // 18
  // public static final int kRearRightMagEncoderPort = 21; // 21
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
