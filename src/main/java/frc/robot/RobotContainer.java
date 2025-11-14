// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.mechanisms.Elevator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // The driver's controller
  public Elevator m_elevator = new Elevator();
  private CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  private CommandXboxController m_coDriver = new CommandXboxController(1);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    // m_driver.back().onTrue(Commands.runOnce (() -> m_robotDrive.zeroGyro()));
    // m_driver.rightStick()
    // .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));
    
    // m_driver.rightTrigger()
    // // .whileTrue(new RunElevator(m_elevator).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = true)).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoXSpeed = true)).alongWith(Commands.runOnce(() -> m_scoringMechPivot.rotatePivot(m_scoringMechPivot.scoringMechGoalAngle)).onlyWhile((() -> m_elevator.getMotorPosition() >= ((327 * m_elevator.percentOfElevator) * .75)))))
    // // .whileFalse(Commands.runOnce(() -> m_scoringMechPivot.rotatePivot(0)).alongWith(Commands.runOnce(() -> m_elevator.rotate(0)).onlyWhile((() -> m_scoringMechPivot.getPositionAngle() >= -20))));
    // .whileTrue(new ElevatorToScore(m_elevator, m_robotDrive, m_scoringMechPivot, m_algae));

    // m_driver.leftTrigger()
    // .whileTrue(new SequentialCommandGroup(Commands.runOnce(() -> Constants.stopNoTwoPieces = true), new RunIntake(m_leds, m_intake, m_intakePivot, m_coral, m_scoringMechSensor, m_algae, new ElevatorToIntakeAlgae(m_elevator, m_robotDrive, m_scoringMechPivot, m_algae), new NoTwoPieces(m_intake, m_intakePivot))) );
 
    // m_driver.y()
    // .whileTrue(new TeleopScore(m_coral, m_elevator, m_intake, m_intakePivot, m_scoringMechSensor, m_algae, m_scoringMechPivot, m_robotDrive).andThen(Commands.waitUntil((() -> m_alignToPoleX.hasReachedX))).finallyDo((() -> {m_robotDrive.isAutoYSpeed = false; m_robotDrive.isAutoXSpeed = false; m_robotDrive.isAutoRotate = RotationEnum.NONE;})));

    // m_driver.x()
    // .whileTrue(new CoralEject(m_intake, m_coral));

    m_driver.a()
    .whileTrue(Commands.runOnce(() -> m_elevator.setVelocity(1)))
    .whileTrue(Commands.runOnce(() -> m_elevator.stopRotate()));
    // m_driver.leftBumper()
    // .whileTrue(Commands.runOnce(() -> m_intake.runIntake(0.3)))
    // .whileFalse(Commands.runOnce(() -> m_intake.stopBoth()));

    // m_driver.pov(90)
    // .whileTrue(new RunIntakeAutoSource(m_intake, m_intakePivot, m_coral, m_scoringMechSensor, m_algae, null, null));

    // m_driver.start()
    // .whileTrue(new ResetEncoders(m_elevator, m_scoringMechPivot, m_intakePivot));

    // m_driver.pov(0)
    // .whileTrue(Commands.runOnce(() -> Constants.visionElevator = !Constants.visionElevator));

    // m_driver.pov(180)
    // .whileTrue(Commands.runOnce(() -> Constants.sensorMode = !Constants.sensorMode));

    // m_driver.pov(270)
    // .whileTrue(Commands.runOnce(() -> Constants.newAlgae = !Constants.newAlgae));

    // m_driver.rightBumper()
    // .whileTrue(Commands.runOnce(() -> Constants.stopNoTwoPieces = true))
    // .whileFalse(Commands.runOnce(() -> Constants.stopNoTwoPieces = false));
    
    // m_coDriver.rightBumper()
    // .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.leftPoint = false));

    // m_coDriver.rightTrigger()
    // .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.leftPoint = false));
    
    // m_coDriver.leftBumper()
    // .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.leftPoint = true));

    // m_coDriver.leftTrigger()
    // .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.leftPoint = true));

    // m_coDriver.rightTrigger()
    // .toggleOnTrue(Commands.runOnce(() -> Constants.scoringMode = "Coral")
    // .alongWith(Commands.runOnce(() -> m_leds.SetSegmentByIntakeMech(Color.kWhite, 50))));

    // m_coDriver.leftTrigger()
    // .toggleOnTrue(Commands.runOnce(() -> Constants.scoringMode = "Algae")
    // .alongWith(Commands.runOnce(() -> m_leds.SetSegmentByIntakeMech(ColorInterface.Algae, 50))));

    // m_driver.b()
    // .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoXSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.NONE))));

    // m_coDriver.a()
    // .whileTrue(new TimerTest());

    // m_coDriver.pov(0)
    // .whileTrue(Commands.runOnce(() -> m_elevator.setScoringPreset(.9, -136, "L4", .49, -260, "Net"))
    // .alongWith(Commands.runOnce(() -> Constants.scoringMode = "Coral")
    // .alongWith(Commands.runOnce(() -> m_leds.SetSegmentByLevel(1.0, ColorInterface.L1, 50)))));
    // m_coDriver.pov(90)
    // .whileTrue(Commands.runOnce(() -> m_elevator.setScoringPreset(.53, 0, "L3", .49, -260, "L3"))
    // .alongWith(Commands.runOnce(() -> Constants.scoringMode = "Coral")
    // .alongWith(Commands.runOnce(() -> m_leds.SetSegmentByLevel(.75, ColorInterface.L1, 50)))));
    // m_coDriver.pov(180)
    // .whileTrue(Commands.runOnce(() -> m_elevator.setScoringPreset(.20, 0, "L1", .30, -260, "Processor"))
    // .alongWith(Commands.runOnce(() -> Constants.scoringMode = "Coral")
    // .alongWith(Commands.runOnce(() -> m_leds.SetSegmentByLevel(.25, ColorInterface.L1, 50)))));
    // m_coDriver.pov(270)
    // .whileTrue(Commands.runOnce(() -> m_elevator.setScoringPreset(.34, 0, "L2", .30, -260, "L2"))
    // .alongWith(Commands.runOnce(() -> Constants.scoringMode = "Coral")
    // .alongWith(Commands.runOnce(() -> m_leds.SetSegmentByLevel(.5, ColorInterface.L1, 50)))));
    // m_coDriver.b()
    // .whileTrue(Commands.runOnce(() -> Constants.scoringMode = "Algae")
    // .alongWith(Commands.runOnce(() -> m_elevator.setScoringPreset(0.49, -260, "L1", 0.49, -260, "L3"))));
    // m_coDriver.x()
    // .whileTrue(Commands.runOnce(() -> Constants.scoringMode = "Algae")
    // .alongWith(Commands.runOnce(() -> m_elevator.setScoringPreset(0.3, -260, "L1", 0.3, -260, "L2"))));

    // m_coDriver.back()
    // .whileTrue(Commands.runOnce(() -> m_elevator.setScoringPreset(.99, -200, "100%", .99, -200, "100%")));
  }

}