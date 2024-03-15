// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualShoot extends Command {
  private IndexerSubsystem indexer;
  DoubleSupplier ampSupplier;
  /**
   * this command forces out a note. All it does is set the indexer to shooting speed. 
   * @param indexer indexer subsystem
   * @param ampSupplier unused, becuase we shouldnt force out a note if we are shooting at the amp
   */
  public ManualShoot(IndexerSubsystem indexer, DoubleSupplier ampSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    this.indexer = indexer;
    this.ampSupplier = ampSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ampSupplier.getAsDouble() == 90 || ampSupplier.getAsDouble() == 45|| ampSupplier.getAsDouble() == 135) {
      indexer.set(IndexerConstants.INDEXER_AMP_SPEED);
    } else {
      indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED);
    }
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
