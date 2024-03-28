// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class AutoGroundIntake extends Command {
  /** Creates a new ConditionalIndexer. */
  IndexerSubsystem indexer;
  IntakeSubsystem intake;
  AngleShooterSubsystem angleShooterSubsystem;
  public AutoGroundIntake(IndexerSubsystem indexer, IntakeSubsystem intake, AngleShooterSubsystem angleShooter) {
    // Use addRequirements() here to declare subsystem dependencies.\
   addRequirements(indexer,intake, angleShooter);
    this.indexer = indexer;
    this.intake = intake;
    this.angleShooterSubsystem = angleShooter;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeYes(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SIDE_SPEED);
    indexer.set(IndexerConstants.INDEXER_INTAKE_SPEED);
    angleShooterSubsystem.setDesiredShooterAngle(ShooterConstants.GROUND_INTAKE_SHOOTER_ANGLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    indexer.off();
    angleShooterSubsystem.setDesiredShooterAngle(angleShooterSubsystem.calculateSpeakerAngle());
    intake.setNoteHeld(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isNoteSeen();
  }
}
