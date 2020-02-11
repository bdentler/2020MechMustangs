/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelManipulator;

public class FlipDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColorWheelManipulator m_wheelManipulator;
  private boolean countReached = false;

  public FlipDown(ColorWheelManipulator subSystem) {
    m_wheelManipulator = subSystem;
    addRequirements(m_wheelManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wheelManipulator.resetFlipperCount();
    countReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_wheelManipulator.flipMotor(MotorSpeeds.kFlipDown) == MotorCounts.kFlipDown) {
        countReached = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wheelManipulator.flipMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return countReached;
  }
}