/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.BallCollector;
import frc.robot.subsystems.ColorWheelManipulator;

public class AutoDriveOffLine extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveOffLine.
   */
  public AutoDriveOffLine(Chassis m_chassis, BallCollector m_sweeper, ColorWheelManipulator m_CP) {
    super(
      new FlipDown(m_CP),
      new DriveStraightAuto(15.0, m_chassis)
      //new DriveStraightAuto(110.0, m_chassis),
      //new GrabberDown(150, m_sweeper),
      //new ShootOutBalls(2.0, m_sweeper),
      //new DriveStraightAuto(-50.0, m_chassis),
      //new DriveStraightAuto(-150.0, m_chassis),
      //new TurnAuto(90.0, m_chassis),
      //new PrintCommand("Grabber counts: " + String.valueOf(m_sweeper.getCount())),
      //new GrabberDown(40, m_sweeper),
      //new StartRoller(m_sweeper)
    );
  }
}
