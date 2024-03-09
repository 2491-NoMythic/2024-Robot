// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class shootNote extends Command {
  IndexerSubsystem indexer;
  AngleShooterSubsystem angleShooter;
  Timer timer;
  double shootTime;
  double revTime;
  /** Creates a new shootNote. */
  public shootNote(IndexerSubsystem indexer, double shootTime, AngleShooterSubsystem angleShooter) {
    this.indexer = indexer;
    this.shootTime = shootTime;
    this.angleShooter = angleShooter;
    timer = new Timer();
    addRequirements(indexer, angleShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    indexer.off();
    angleShooter.setDesiredShooterAngle(angleShooter.calculateSpeakerAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>=0.8) {
      indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED);
    }
    SmartDashboard.putNumber("auto timer", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>shootTime;
  }
}
