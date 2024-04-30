// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class InitialShot extends Command {
  ShooterSubsystem shooter;
  IndexerSubsystem indexer;
  Timer timer;
  double revTime;
  double shootTime;
  AngleShooterSubsystem angleShooter;
  boolean shot;
  /** Creates a new shootThing. */
  public InitialShot(ShooterSubsystem shooter, IndexerSubsystem indexer, double revTime, double shootTime, AngleShooterSubsystem angleShooter) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.revTime = revTime;
    this.shootTime = shootTime;
    this.angleShooter = angleShooter;
    timer = new Timer();
    addRequirements(shooter, indexer, angleShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shot = false;
    timer.reset();
    timer.start();
    indexer.off();
    shooter.setTargetVelocity(65, 50, 600, 600);
    SmartDashboard.putNumber("shooter resets", SmartDashboard.getNumber("shooter resets", 0)+1);
    angleShooter.setDesiredShooterAngle(Field.SUBWOOFER_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getRSpeed()>35&&shooter.getLSpeed()>60) {
      indexer.on();
      timer.reset();
      shot = true;
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
    return timer.get()>shootTime&&shot;

  }
}
