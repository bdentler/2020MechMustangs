/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.commandStick;
import frc.robot.Constants.driveStick;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ColorWheelManipulator;
import frc.robot.subsystems.BallCollector;
import frc.robot.subsystems.Winch;
import frc.robot.commands.AutoDriveOffLine;
import frc.robot.commands.AutoDropRetreat;

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
 
  private final Chassis m_chassis = new Chassis();
  private final ColorWheelManipulator m_colorWheel = new ColorWheelManipulator();
  private final BallCollector m_ballCollector = new BallCollector();
  private final Winch m_winch = new Winch();
  
  XboxController m_commandController = new XboxController(commandStick.kCommandStickPort);
  Joystick m_driveController = new Joystick(driveStick.kDriveStickPort);

  // this defines an autonomous command - return the command below
  private final AutoDropRetreat m_complex = new AutoDropRetreat(m_colorWheel, m_chassis, m_ballCollector);
  private final AutoDriveOffLine m_simple = new AutoDriveOffLine(m_chassis, m_ballCollector, m_colorWheel);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {

    m_chassis.setDefaultCommand(
      // drive the robot with a Logitech Joystick
      // forward/backward moves robot forward/backward
      // left/right makes left and right turns
      
      new RunCommand(() -> m_chassis
          .driveChassis(m_driveController.getY(),
                       m_driveController.getX()), m_chassis));
    
                       // Add commands to the autonomous command chooser
    m_chooser.addOption("Drive and Keep balls", m_simple);
    m_chooser.addOption("Drive and Dump balls", m_complex);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driveController, driveStick.kDriveStickButton9)
        .whenPressed(() -> m_chassis.setMaxOutput(0.5));

    new JoystickButton(m_driveController, driveStick.kDriveStickButton7)
        .whenPressed(() -> m_chassis.setMaxOutput(1.0));

    new JoystickButton(m_driveController, driveStick.kDriveStickTrigger)
        .whenPressed(() -> m_ballCollector.rollerMotor(MotorSpeeds.kRollOut))
        .whenReleased(() -> m_ballCollector.rollerMotor(MotorSpeeds.kRollIn));

    new POVButton(m_commandController, commandStick.kCommandStickPOVDown)
        .whenPressed(() -> m_colorWheel.flipMotor(MotorSpeeds.kFlipDown))
        .whenReleased(() -> m_colorWheel.flipMotor(0));
    
    new POVButton(m_commandController, commandStick.kCommandStickPOVUp)
        .whenPressed(() -> m_colorWheel.flipMotor(MotorSpeeds.kFlipUp))
        .whenReleased(() -> m_colorWheel.flipMotor(0));
  
    new POVButton(m_driveController, driveStick.kDriveStickPOVDown)
        .whenPressed(() -> m_ballCollector.liftMotor(MotorSpeeds.kLiftUp))
        .whenReleased(() -> m_ballCollector.liftMotor(0));
    
    new POVButton(m_driveController, driveStick.kDriveStickPOVUp)
        .whenPressed(() -> m_ballCollector.liftMotor(MotorSpeeds.kLowerDown))
        .whenReleased(() -> m_ballCollector.liftMotor(0));

    new JoystickButton(m_commandController, commandStick.kButtonLB)
        .whenPressed(() -> m_winch.climb(MotorSpeeds.kWinchLift))
        .whenReleased(() -> m_winch.climb(0));
    
    new JoystickButton(m_commandController, commandStick.kButtonRB)
        .whenPressed(() -> m_winch.climb(MotorSpeeds.kWinchExtend))
        .whenReleased(() -> m_winch.climb(0));

    new JoystickButton(m_commandController, commandStick.kButtonY)
        .whenPressed(() -> m_colorWheel.rotateWheel(MotorSpeeds.kRotateWheel))
        .whenReleased(() -> m_colorWheel.rotateWheel(0));
    
    new JoystickButton(m_commandController, commandStick.kButtonA)
        .whenPressed(() -> m_ballCollector.rollerMotor(0))
        .whenReleased(() -> m_ballCollector.rollerMotor(0));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}


