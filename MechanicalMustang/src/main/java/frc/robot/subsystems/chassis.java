/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.PWM;

public class chassis extends SubsystemBase {
  PWMVictorSPX leftDriveMotors = null;
  PWMVictorSPX rightDriveMotors = null;
  DifferentialDrive chassis = null;

  public chassis() {
    leftDriveMotors = new PWMVictorSPX(PWM.kLeftDriveMotors);
    rightDriveMotors = new PWMVictorSPX(PWM.kRightDriveMotors);
    chassis = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
  }

  public void driveChassis(double driveSpeed, double driveRotation) {
    chassis.arcadeDrive(driveSpeed, driveRotation);
  }

  public void setMaxOutput(double maxOutput) {
    chassis.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
