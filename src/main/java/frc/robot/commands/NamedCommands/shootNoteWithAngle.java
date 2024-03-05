// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class shootNoteWithAngle extends Command {
  IndexerSubsystem indexer;
  ShooterSubsystem shooter;
  AngleShooterSubsystem angleShooter;
  Timer timer;
  double shootTime;
  double revTime;
  double angle;
  /** Creates a new shootNote. */
  public shootNoteWithAngle(IndexerSubsystem indexer, ShooterSubsystem shooter, AngleShooterSubsystem angleShooter, double shootTime, double revTime, double angle) {
    this.angleShooter = angleShooter;
    this.indexer = indexer;
    this.shootTime = shootTime;
    this.revTime = revTime;
    this.shooter = shooter;
    this.angle = angle;
    timer = new Timer();
    addRequirements(indexer, shooter, angleShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.shootRPS(ShooterConstants.SHORT_SHOOTING_RPS);
    angleShooter.setDesiredShooterAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>revTime) {
      indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    indexer.off();
    shooter.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>shootTime;
  }
}
