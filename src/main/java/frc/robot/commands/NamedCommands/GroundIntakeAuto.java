// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GroundIntakeAuto extends Command {
  /** Creates a new GroundIntake. */
  IntakeSubsystem intake;
  IndexerSubsystem indexer;
  AngleShooterSubsystem angleShooter;
  public GroundIntakeAuto(IntakeSubsystem intake, IndexerSubsystem indexer, AngleShooterSubsystem angleShooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.angleShooter = angleShooter;
    addRequirements(intake, indexer, angleShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleShooter.setDesiredShooterAngle(ShooterConstants.GROUND_INTAKE_SHOOTER_ANGLE);
    intake.intakeYes(IntakeConstants.INTAKE_SPEED);
    indexer.set(IndexerConstants.INDEXER_INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isNoteIn();
  }
}
