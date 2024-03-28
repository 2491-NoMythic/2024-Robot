// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command {
  /** Creates a new WaitCommand. */
  DoubleSupplier m_waitTime;
  Timer m_timer;
  

  public WaitCommand(DoubleSupplier waitTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_waitTime = waitTime;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_timer.reset();
      m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_waitTime.getAsDouble();
  }
}
