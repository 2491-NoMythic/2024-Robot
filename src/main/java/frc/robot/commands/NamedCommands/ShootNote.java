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
import frc.robot.subsystems.IntakeSubsystem;

public class ShootNote extends Command {
  IndexerSubsystem indexer;
  Timer timer;
  double shootTime;
  double revTime;
  IntakeSubsystem intake;
  /** Creates a new shootNote. */
  public ShootNote(IndexerSubsystem indexer, double shootTime, double revTime, IntakeSubsystem intake) {
    SmartDashboard.putNumber("notes shot", 0);
    this.indexer = indexer;
    this.shootTime = shootTime;
    this.intake = intake;
    this.revTime = revTime;
    timer = new Timer();
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("notes shot", SmartDashboard.getNumber("notes shot", 0)+1);
    timer.reset();
    timer.start();
    indexer.off();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>revTime) {
      indexer.setVoltage(12);
    }
    SmartDashboard.putNumber("auto timer", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    indexer.off();
    timer.reset();
    intake.setNoteHeld(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>shootTime;
  }
}
