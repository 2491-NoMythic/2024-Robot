// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ClimberConstants;

public class RunClimberDown extends Command {
  public Climber climber;
  private double speed;
  /** Creates a new RunClimber. */
  public RunClimberDown(double speed, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Climb start

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Climber go climbing
    climber.climberGo(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Climber no climbing
    climber.climberGo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
