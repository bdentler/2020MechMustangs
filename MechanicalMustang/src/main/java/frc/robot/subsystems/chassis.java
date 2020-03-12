/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants.PWM;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PID;

public class Chassis extends SubsystemBase {
  PWMVictorSPX leftDriveMotors = null;
  PWMVictorSPX rightDriveMotors = null;
  DifferentialDrive chassis = null;
  Encoder leftDriveEncoder = null;
  Encoder rightDriveEncoder = null;
  Gyro driveGyro = null;

  public Chassis() {
    leftDriveMotors = new PWMVictorSPX(PWM.kLeftDriveMotors);
    rightDriveMotors = new PWMVictorSPX(PWM.kRightDriveMotors);
    chassis = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
    leftDriveEncoder = new Encoder(DIO.kLeftEncoderPorts[0], DIO.kLeftEncoderPorts[1], DIO.kLeftEncoderReversed);
    rightDriveEncoder = new Encoder(DIO.kRightEncoderPorts[0], DIO.kRightEncoderPorts[1], DIO.kRightEncoderReversed);
    driveGyro = new ADXRS450_Gyro();

    leftDriveEncoder.setDistancePerPulse(DIO.kEncoderDistancePerPulse);
    rightDriveEncoder.setDistancePerPulse(DIO.kEncoderDistancePerPulse);
    driveGyro.calibrate();
    
  }

  public void resetEncoders() {
    leftDriveEncoder.reset();
    rightDriveEncoder.reset();
  }

  public void calibrateGyro() {
    driveGyro.calibrate();
  }

  public void resetGyro() {
    driveGyro.reset();
  }

  public double getHeading() {
    return driveGyro.getAngle();
  }

  public double getRightEncoderDistance() {
    return rightDriveEncoder.getDistance();
  }

  public double getLeftEncoderDistance() {
    return leftDriveEncoder.getDistance();
  }

  public double getAverageEncoderDistance() {
    return (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance()) / 2.0;
  }

  public void driveChassis(double driveSpeed, double driveRotation) {
    chassis.arcadeDrive(-driveSpeed, driveRotation, false);
  }

  public void setMaxOutput(double maxOutput) {
    chassis.setMaxOutput(maxOutput);
  }

  public boolean driveStraight(double speed, double distance) {
    boolean arrived = false;
    double headingError = -getHeading();
    double rotation = PID.kPAngle * headingError;
    double distanceError = distance - getAverageEncoderDistance();
    if (distanceError > 10) {
      speed = distanceError / 10 * speed * PID.kPDrive;
    } else {
      speed = 0;
      arrived = true;
    }
    chassis.arcadeDrive(speed, rotation, false);
    return arrived;
  }

  public boolean rotateToAngle(double targetAngle) {
    double error = targetAngle - getHeading();
    double rotation;
    boolean returnValue = false;
    if (error > PID.rotateToAngleThreshHold) {
      rotation = error * PID.kPAngle;
    } else {
      rotation = 0;
      returnValue = true;
    }
    chassis.arcadeDrive(0, rotation, false);
    return returnValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
