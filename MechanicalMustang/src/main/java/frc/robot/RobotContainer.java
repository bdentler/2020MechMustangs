/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.ColorWheelManipulator;
import frc.robot.commands.FlipUp;
import frc.robot.commands.FlipDown;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.chassis;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.commandStick;
import frc.robot.Constants.driveStick;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   * 1. Instantiate all individual Subsystems
   * 2. Instantiate all components required to interact with the subsytem (controllers,
   *    actuators, camera pipelines, etc.)
   * 3. Define subsystem default commands, passing any needed dependencies to the commands
   * 4. Map button bindings for non-default commands
   */
 
  private final chassis m_chassis = new chassis();
  private final ColorWheelManipulator m_colorWheel = new ColorWheelManipulator();


  XboxController m_commandController = new XboxController(commandStick.kCommandStickPort);
  Joystick m_driveController = new Joystick(driveStick.kDriveStickPort);

  // this defines an autonomous command - return the command below
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final FlipUp m_flipUp = new FlipUp(m_colorWheel);
  private final FlipDown m_flipDown = new FlipDown(m_colorWheel);

  public RobotContainer() {

    m_chassis.setDefaultCommand(
      // drive the robot with a Logitech Joystick
      // forward/backward moves robot forward/backward
      // left/right makes left and right turns
      
      new RunCommand(() -> m_chassis
          .driveChassis(m_driveController.getY(),
                       m_driveController.getX()), m_chassis));
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, driveStick.kDriveStickTrigger)
        .whenPressed(() -> m_chassis.setMaxOutput(0.5))
        .whenReleased(() -> m_chassis.setMaxOutput(1));
    
    new JoystickButton(m_commandController, commandStick.kFlipUp)
        .whenPressed(m_flipUp);
    
    new JoystickButton(m_commandController, commandStick.kFlipDown)
        .whenPressed(m_flipDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  //}
}


