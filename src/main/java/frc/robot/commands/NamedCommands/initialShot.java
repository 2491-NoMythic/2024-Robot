// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class initialShot extends Command {
  ShooterSubsystem shooter;
  IndexerSubsystem indexer;
  Timer timer;
  double revTime;
  double shootTime;
  /** Creates a new shootThing. */
  public initialShot(ShooterSubsystem shooter, IndexerSubsystem indexer, double revTime, double shootTime) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.revTime = revTime;
    this.shootTime = shootTime;
    timer = new Timer();
    addRequirements(shooter, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    indexer.off();
    shooter.shootRPS(ShooterConstants.SHORT_SHOOTING_RPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>revTime) {
      indexer.on();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>shootTime;
  }
}
