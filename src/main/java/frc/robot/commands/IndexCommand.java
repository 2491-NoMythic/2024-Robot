// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.ShooterSubsystem;


public class IndexCommand extends Command {
  BooleanSupplier revUpSupplier;
  Boolean revUp;
  BooleanSupplier shootIfReadySupplier;
  Boolean shootIfReady;

  IndexerSubsystem m_Indexer;
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  DrivetrainSubsystem drivetrain;
  AngleShooterSubsystem angleShooterSubsytem;
  boolean auto;

  /** Creates a new IndexCommand. */
  public IndexCommand(IndexerSubsystem m_IndexerSubsystem, BooleanSupplier shootIfReadySupplier, BooleanSupplier revUpSupplier, ShooterSubsystem shooter, IntakeSubsystem intake, DrivetrainSubsystem drivetrain, AngleShooterSubsystem angleShooterSubsystem) {
    this.m_Indexer = m_IndexerSubsystem;
    this.shootIfReadySupplier = shootIfReadySupplier;
    this.revUpSupplier = revUpSupplier;
    this.shooter = shooter;
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.angleShooterSubsytem = angleShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IndexerSubsystem, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isAutonomousEnabled()) {
      auto = true;
    } else {
      auto = false;
    }
    if (!intake.isNoteIn()) {
      intake.intakeYes(IntakeConstants.INTAKE_SPEED);
      m_Indexer.set(IndexerConstants.INDEXER_INTAKE_SPEED);
      if(!auto) {
        shooter.turnOff();
      }
    } else {
      intake.intakeOff();
      if(revUpSupplier.getAsBoolean()) {
        shooter.shootRPS(ShooterConstants.SHOOTING_RPS);
      } else {
        shooter.shootRPS(ShooterConstants.AMP_RPS);
      }
    boolean indexer = false;
    if(angleShooterSubsytem.validShot() && drivetrain.validShot() && shooter.validShot()) {
      RobotState.getInstance().ShooterReady = true;
      if (shootIfReadySupplier.getAsBoolean()) {
        indexer = true;
      }
    } else {
      RobotState.getInstance().ShooterReady = false;
    }
    if (indexer) {
      m_Indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED);
    } else {
      m_Indexer.off();
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().ShooterReady = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
