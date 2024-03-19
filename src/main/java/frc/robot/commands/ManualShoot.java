// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
This command sets the indexer to one of two speeds, depending on the D-pad(90, 45 and 135 = INDEXER_AMP_SPEED), otherwise speed is equal
 to INDEXER_SHOOTING_SPEED
 **/
public class ManualShoot extends Command {
  private IndexerSubsystem indexer;
  DoubleSupplier ampSupplier;
  IntakeSubsystem intake;
  /** Creates a new ManualShoot. 
   * This command sets the indexer to one of two speeds, depending on the D-pad(90, 45 and 135 = INDEXER_AMP_SPEED), otherwise speed is equal
     to INDEXER_SHOOTING_SPEED. 
  */
  public ManualShoot(IndexerSubsystem indexer, DoubleSupplier ampSupplier, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, intake);
    this.indexer = indexer;
    this.ampSupplier = ampSupplier;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.set(IndexerConstants.INDEXER_SHOOTING_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.off();
    intake.setNoteHeld(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
