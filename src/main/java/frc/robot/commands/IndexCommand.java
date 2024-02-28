// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.ShooterConstants.AMP_RPS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.ShooterConstants;
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
  DoubleSupplier ampSupplier;;
  Boolean shootIfReady;
  // DoubleSupplier POVSupplier;
  BooleanSupplier humanPlayerSupplier;
  IndexerSubsystem m_Indexer;
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  DrivetrainSubsystem drivetrain;
  AngleShooterSubsystem angleShooterSubsytem;
  BooleanSupplier stageAngleSup;
  BooleanSupplier subwooferAngleSup;
  boolean auto;
  double runsEmpty = 0;

  /** Creates a new IndexCommand. */
  public IndexCommand(IndexerSubsystem m_IndexerSubsystem, BooleanSupplier shootIfReadySupplier, BooleanSupplier revUpSupplier, ShooterSubsystem shooter, IntakeSubsystem intake, DrivetrainSubsystem drivetrain, AngleShooterSubsystem angleShooterSubsystem, BooleanSupplier humanPlaySupplier, BooleanSupplier stageAngleSup, BooleanSupplier SubwooferSup) {
    this.m_Indexer = m_IndexerSubsystem;
    this.shootIfReadySupplier = shootIfReadySupplier;//R2
    this.revUpSupplier = revUpSupplier;//L2
    this.shooter = shooter;
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.angleShooterSubsytem = angleShooterSubsystem;
    this.humanPlayerSupplier = humanPlaySupplier;//R1
    this.subwooferAngleSup = SubwooferSup;
    this.stageAngleSup = stageAngleSup;
    SmartDashboard.putNumber("amp RPS", AMP_RPS);
    SmartDashboard.putNumber("indexer amp speed", IndexerConstants.INDEXER_AMP_SPEED);
    SmartDashboard.putNumber("amp angle", Field.AMPLIFIER_ANGLE);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IndexerSubsystem, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runsEmpty = 0;
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
      // intake.intakeYes(IntakeConstants.INTAKE_SPEED); // only code that runs the intake
      if(runsEmpty<21) {runsEmpty++;}
      if(runsEmpty>20) {
        if(humanPlayerSupplier.getAsBoolean()) {
          m_Indexer.set(IndexerConstants.HUMAN_PLAYER_INDEXER_SPEED);
          shooter.shootRPS(ShooterConstants.HUMAN_PLAYER_RPS);
          intake.intakeOff();
        }
        else {
          m_Indexer.set(IndexerConstants.INDEXER_INTAKE_SPEED);
          shooter.turnOff();
        }
      }
    } else {
      runsEmpty = 0;
      intake.intakeOff();
      if(shootIfReadySupplier.getAsBoolean()&&revUpSupplier.getAsBoolean()&&humanPlayerSupplier.getAsBoolean()) {
        shooter.shootRPS(ShooterConstants.SUBWOOFER_RPS);
      } else {
        if(revUpSupplier.getAsBoolean()||stageAngleSup.getAsBoolean()||subwooferAngleSup.getAsBoolean()) {
          if(angleShooterSubsytem.shortSpeakerDist()) {
            shooter.shootRPS(ShooterConstants.SHORT_SHOOTING_RPS);
          } else {
            shooter.shootRPS(ShooterConstants.LONG_SHOOTING_RPS);
          }
        } else {
          shooter.shootRPS(SmartDashboard.getNumber("amp RPS", ShooterConstants.AMP_RPS)/*ShooterConstants.AMP_RPS*/);
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
        if(SmartDashboard.getBoolean("feedMotor", false)) {
          indexer = true;
        }
        if (indexer) {
            m_Indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED);
         } else {
            m_Indexer.off();
         }
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
