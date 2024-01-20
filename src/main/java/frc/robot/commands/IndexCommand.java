// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IndexCommand extends Command {
  BooleanSupplier shootButtonSupplier;
  Boolean shootButton;
  IndexerSubsystem m_Indexer;
  ShooterSubsystem shooter;
  IntakeSubsystem intake;

  /** Creates a new IndexCommand. */
  public void IndexCommand(IndexerSubsystem m_IndexerSubsystem, BooleanSupplier shootButtonSupplier, ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.m_Indexer = m_IndexerSubsystem;
    this.shootButtonSupplier = shootButtonSupplier;
    this.shooter = shooter;
    this.intake = intake;

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
    if (Sensor.noseeing()) {
      intake.intakeYes(1);
      m_Indexer.holderRetrieve(0.5);
    } else {
      intake.intakeOff();
      m_Indexer.holderOff();
    }
    if (shootButtonSupplier.getAsBoolean()) {
      shooter.shootThing(1);
      m_Indexer.feederFeed(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
