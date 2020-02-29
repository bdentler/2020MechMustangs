/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Counter;

import frc.robot.Constants.PWM;
import frc.robot.Constants.DIO;
import frc.robot.Constants.MotorCounts;

public class BallCollector extends SubsystemBase {
  Spark liftMotor = null;
  Spark rollerMotor = null;

  Counter liftCount = new Counter();
  double lastRollerSpeed = 0;
  int lastRollerSpeedCount = 0;
  
  public BallCollector() {
    liftMotor = new Spark(PWM.kBallLift);
    rollerMotor = new Spark(PWM.kBallRoller);

    liftCount.setUpSource(DIO.kLiftMotor);
    liftCount.setUpDownCounterMode();
  }

  public void resetLiftCount() {
    liftCount.reset();
  }

  public int liftMotor(double speed) {
    liftMotor.setSpeed(speed);
    return liftCount.get();
  }

  public void rollerMotor(double speed) {
    if (lastRollerSpeed != speed) {
      lastRollerSpeedCount += 1;
      if (lastRollerSpeedCount >= MotorCounts.kRollerDecceleration) {
        lastRollerSpeedCount = 0;
        lastRollerSpeed = speed;
      } else {
        speed = 0;
      }
    }
    rollerMotor.setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
