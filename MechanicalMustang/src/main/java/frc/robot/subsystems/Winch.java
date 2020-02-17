/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;

import frc.robot.Constants.PWM;

public class Winch extends SubsystemBase {
  Spark climbMotor = null;
  Spark crawlMotor = null;

  public Winch() {
    climbMotor = new Spark(PWM.kClimb);
    crawlMotor = new Spark(PWM.kCrawl);

  }

  public void climb(double speed) {
    climbMotor.setSpeed(speed);
  }

  public void crawl(double speed) {
    crawlMotor.setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
