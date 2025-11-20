// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

public class sensorAbsEnc {
  // Swerve module absolute encoder (wheel angle)
  private final CANcoder m_canCoder;
  private final double m_canCoderOffsetDegrees;

  public sensorAbsEnc(int magEncoderID, double magEncoderOffsetDegrees) {
    // Create steering encoder objects (high resolution encoder)
    m_canCoder = new CANcoder(magEncoderID, "rio");
    m_canCoderOffsetDegrees = magEncoderOffsetDegrees;

    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    toApply.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    m_canCoder.getPosition().setUpdateFrequency(100);
    m_canCoder.getVelocity().setUpdateFrequency(100);

  }

  /**
   * Returns the CANcoder's measured turn angle in degrees.
   */
  public double getCanCoder() {
    var posVal = m_canCoder.getPosition();
    if(posVal.getStatus().isOK()) {
        double val = posVal.getValueAsDouble();
        return val * 360.0;
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
        return 0.0;
    }
  }

// Write code with the following hint to correct the initialization of the turning encoder position!
// m_turningEncoder.setPosition(Units.degreesToRadians(m_canCoder.getAbsolutePosition().getValueAsDouble() * 360.0 - m_canCoderOffsetDegrees));    

    //   // According to this:
    // // https://www.chiefdelphi.com/t/ctre-phoenix-pro-to-phoenix-6-looking-back-and-looking-ahead/437313/27
    // // When using Phoenix6, the CanCoder should not have problems on startup as long as we wait for an update and check for errors.
    // var posVal = m_canCoder.getAbsolutePosition().waitForUpdate(0.1); // This actaully waits that long! Don't call after init!
    // if(posVal.getStatus().isOK()) {
    //     /* Perform seeding */
    //     double val = posVal.getValueAsDouble();
    //     m_turningEncoder.setPosition(Units.degreesToRadians(val * 360.0 - m_canCoderOffsetDegrees));
    // } else {
    //     /* Report error and retry later */
    //     System.out.println("Error reading CANcoder position! Robot will not drive straight!");
    // }

}
