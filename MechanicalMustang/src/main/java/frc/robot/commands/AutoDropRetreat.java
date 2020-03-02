/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.chassis;
import frc.robot.subsystems.BallCollector;
import frc.robot.subsystems.ColorWheelManipulator;

public class AutoDropRetreat extends SequentialCommandGroup {

  public AutoDropRetreat(chassis chassis, BallCollector ballCollector, ColorWheelManipulator CP) {

    addCommands(
      parallel(
        new GrabberDown(10, ballCollector),
        new DriveStraightAuto(36.0, chassis)),
      
      new ShootOutBalls(2.0, ballCollector),
      
      parallel(
        new FlipDown(CP),
        new DriveStraightAuto(-36.0, chassis),
        new GrabberDown(30, ballCollector)),
      
      new StartRoller(ballCollector)
    );
  }
}
