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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.Constants.*;

public class ColorWheelManipulator extends SubsystemBase {
  final I2C.Port i2cPort = I2C.Port.kOnboard;
  Spark flipperMotor = null;
  Spark rotatorMotor = null;
  ColorSensorV3 colorSensor = null;
  ColorMatch colorMatcher = null;

  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  Counter flipperCount = new Counter();
  
  public ColorWheelManipulator() {
    flipperMotor = new Spark(PWM.kFlipperMotor);
    rotatorMotor = new Spark(PWM.kRotatorMotor);
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget); 

    flipperCount.setUpSource(DIO.kFlipperMotor);
    flipperCount.setUpDownCounterMode();
  }

  public void resetFlipperCount() {
    flipperCount.reset();
  }

  public int flipMotor(double speed) {
    flipperMotor.setSpeed(speed);
    return flipperCount.get();
  }

  public void rotateWheel(double speed) {
    rotatorMotor.setSpeed(speed);
  }

  public String getCurrentColor() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    String result = "Unknown";

    if (match.color == kBlueTarget) {
      result = "Blue";
    } else if (match.color == kRedTarget) {
      result = "Red";
    } else if (match.color == kGreenTarget) {
      result = "Green";
    } else if (match.color == kYellowTarget) {
      result = "Yellow";
    }
    
    return result;
  }
  public boolean colorMatches(String Color) {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    boolean result = false;

    switch(Color) {
      case "Blue": 
        if (match.color == kBlueTarget) {
          result = true;
        }
      case "Red":
        if (match.color == kRedTarget) {
          result = true;
        }
      case "Green":
        if (match.color == kGreenTarget) {
          result = true;
        }
      case "Yellow":
        if (match.color == kYellowTarget) {
          result = true;
        }
      default:
        result = false;
    }
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
