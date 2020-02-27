/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.ColorWheelManipulator;
import frc.robot.subsystems.BallCollector;
import frc.robot.subsystems.Winch;
//import frc.robot.commands.FlipUp;
//import frc.robot.commands.FlipDown;
import frc.robot.commands.AutoDriveOffLine;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.chassis;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.MotorSpeeds;
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
  private final BallCollector m_ballCollector = new BallCollector();
  private final Winch m_winch = new Winch();
  
  XboxController m_commandController = new XboxController(commandStick.kCommandStickPort);
  Joystick m_driveController = new Joystick(driveStick.kDriveStickPort);

  // this defines an autonomous command - return the command below
  private final AutoDriveOffLine m_autoCommand = new AutoDriveOffLine(m_chassis);
  //private final FlipUp m_flipUp = new FlipUp(m_colorWheel);
  //private final FlipDown m_flipDown = new FlipDown(m_colorWheel);

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
    
    new JoystickButton(m_commandController, commandStick.kButtonB)
        .whileHeld(() -> m_colorWheel.flipMotor(MotorSpeeds.kFlipUp))
        .whenReleased(() -> m_colorWheel.flipMotor(0));
    
    new JoystickButton(m_commandController, commandStick.kButtonA)
        .whileHeld(() -> m_colorWheel.flipMotor(MotorSpeeds.kFlipDown))
        .whenReleased(() -> m_colorWheel.flipMotor(0));
  
    new JoystickButton(m_commandController, commandStick.kButtonY)
        .whileHeld(() -> m_ballCollector.liftMotor(MotorSpeeds.kLiftUp))
        .whenReleased(() -> m_ballCollector.liftMotor(0));
    
    new JoystickButton(m_commandController, commandStick.kButtonX)
        .whileHeld(() -> m_ballCollector.liftMotor(MotorSpeeds.kLowerDown))
        .whenReleased(() -> m_ballCollector.liftMotor(0));

    new JoystickButton(m_commandController, commandStick.kButtonLB)
        .whileHeld(() -> m_winch.climb(MotorSpeeds.kWinchLift))
        .whenReleased(() -> m_winch.climb(0));
    
    new JoystickButton(m_commandController, commandStick.kButtonRB)
        .whileHeld(() -> m_winch.climb(MotorSpeeds.kWinchExtend))
        .whenReleased(() -> m_winch.climb(0));

    new JoystickButton(m_commandController, commandStick.kButtonBack)
        .whileHeld(() -> m_colorWheel.rotateWheel(MotorSpeeds.kRotateWheel))
        .whenReleased(() -> m_colorWheel.rotateWheel(0));
    
    new JoystickButton(m_commandController, commandStick.kButtonStart)
        .whileHeld(() -> m_ballCollector.rollerMotor(MotorSpeeds.kRollIn))
        .whenReleased(() -> m_ballCollector.rollerMotor(0));
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}


