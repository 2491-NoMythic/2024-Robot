// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberPullDown extends Command {
  /** Creates a new ClimberDown. */
  Climber m_climber;
  
  public ClimberPullDown(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_climber.climberVoltage(ClimberConstants.CLIMBER_VOLTAGE_PULL_DOWN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.isClimberIn();
  }
}
