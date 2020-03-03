/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.subsystems.Chassis;

public class DriveStraightAuto extends CommandBase {
  Chassis m_chassis;
  double m_distance;
  double m_heading;
  double m_direction;
  
  public DriveStraightAuto(double distance, Chassis subsys) {
    m_chassis = subsys;
    m_distance = distance;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.driveChassis(0, 0);
    m_chassis.resetEncoders();
    m_chassis.resetGyro();
    m_heading = m_chassis.getHeading();
    m_direction = Math.abs(m_distance) / m_distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = 0;
    double currentHeading = m_chassis.getHeading();
    if (currentHeading - m_heading > 2.0) {
      rot = -0.1;
    } else if (currentHeading - m_heading < 2.0) {
      rot = 0.1;
    }
    m_chassis.driveChassis(m_direction * MotorSpeeds.kAutoDriveSpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.driveChassis(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_chassis.getAverageEncoderDistance()) >= m_distance);
  }
}
