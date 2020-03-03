/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.BallCollector;
import frc.robot.subsystems.ColorWheelManipulator;

public class AutoDropRetreat extends SequentialCommandGroup {

  public AutoDropRetreat(ColorWheelManipulator m_CP, Chassis m_chassis, BallCollector m_sweeper) {

    super(
      //new FlipDown(m_CP),
      new DriveStraightAuto(36.0, m_chassis),
      //new GrabberDown(10, m_sweeper),
      new ShootOutBalls(2.0, m_sweeper),
      new PrintCommand("start drive straight auto going backward"),
      new DriveStraightAuto(-36.0, m_chassis),
      new StartRoller(m_sweeper)
    );
  }
}
