/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.chassis;
/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while this
 * command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class DriveStraightAuto extends PIDCommand {
  private final chassis m_chassis;
  public DriveStraightAuto(double distance, chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(new PIDController(4, 0, 0),
        chassis::getAverageEncoderDistance,
        distance,
        d -> chassis.driveChassis(d, d));
      
    m_chassis = chassis;
    addRequirements(m_chassis);
    getController().setTolerance(0.01);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.resetEncoders();
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
