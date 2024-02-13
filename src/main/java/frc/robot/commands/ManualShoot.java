// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualShoot extends Command {
  private IndexerSubsystem indexer;
  /** Creates a new ManualShoot. */
  public ManualShoot(IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.on();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
