/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis;

public class AutoDriveOffLine extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final chassis m_chassis;
  /**
   * Creates a new AutoDriveOffLine.
   */
  public AutoDriveOffLine(chassis subSystem) {
    m_chassis = subSystem;
    addRequirements(m_chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.driveChassis(0, 0);
    m_chassis.resetEncoders();
    m_chassis.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = m_chassis.getHeading();
    double rot;
    if (heading < -2.0) {
      rot = 0.1;
    } else if (heading > 2.0) {
      rot = -0.1;
    } else {
      rot = 0;
    }
    m_chassis.driveChassis(0.5, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.driveChassis(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_chassis.getAverageEncoderDistance() < 36.0);
  }
}
