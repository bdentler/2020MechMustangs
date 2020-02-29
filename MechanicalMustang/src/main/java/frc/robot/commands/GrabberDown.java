/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCollector;
import frc.robot.Constants.MotorCounts;
import frc.robot.Constants.MotorSpeeds;

public class GrabberDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final BallCollector m_ballCollector;
  private boolean countReached = false;

  public GrabberDown(BallCollector subsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_ballCollector = subsystem;
    addRequirements(m_ballCollector);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_ballCollector.resetLiftCount();;
    countReached = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (m_ballCollector.liftMotor(MotorSpeeds.kLowerDown) == MotorCounts.kLowerDown) {
      countReached = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballCollector.liftMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return countReached;
  }
}