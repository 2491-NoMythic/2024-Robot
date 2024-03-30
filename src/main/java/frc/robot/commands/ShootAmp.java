// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmp extends Command {
  IndexerSubsystem indexer;
  ShooterSubsystem shooter;
  Timer timer;
  /** Creates a new shootAmp. */
  public ShootAmp(IndexerSubsystem indexer, ShooterSubsystem shooter) {
    addRequirements(shooter, indexer);
    this.indexer = indexer;
    this.shooter = shooter;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.shootSameRPS(ShooterConstants.PRAC_AMP_RPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Preferences.getBoolean("CompBot", true)){
      if(timer.get()>1) {
        indexer.set(IndexerConstants.COMP_INDEXER_AMP_SPEED);
    }
  }
  else{
    if(timer.get()>1){
      indexer.set(IndexerConstants.PRAC_INDEXER_AMP_SPEED);
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.turnOff();
    indexer.off();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>1.5;
  }
}
