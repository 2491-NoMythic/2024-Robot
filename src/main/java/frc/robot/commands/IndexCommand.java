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
import frc.robot.settings.Constants.IntakeConstants;
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
  DoubleSupplier ampSupplier;
  BooleanSupplier groundIntakeSup;
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
  BooleanSupplier farStageAngleSup;
  BooleanSupplier operatorRevSup;
  boolean auto;
  double runsEmpty = 0;

  /** Creates a new IndexCommand. */
  public IndexCommand(IndexerSubsystem m_IndexerSubsystem, BooleanSupplier shootIfReadySupplier, BooleanSupplier revUpSupplier, ShooterSubsystem shooter, IntakeSubsystem intake, DrivetrainSubsystem drivetrain, AngleShooterSubsystem angleShooterSubsystem, BooleanSupplier humanPlaySupplier, BooleanSupplier stageAngleSup, BooleanSupplier SubwooferSup, BooleanSupplier groundIntakeSup, BooleanSupplier farStageAngleSup, BooleanSupplier operatorRevSup) {
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
    this.farStageAngleSup = farStageAngleSup;
    this.groundIntakeSup = groundIntakeSup;
    this.operatorRevSup = operatorRevSup;
    SmartDashboard.putNumber("amp RPS", AMP_RPS);
    SmartDashboard.putNumber("indexer amp speed", IndexerConstants.INDEXER_AMP_SPEED);
    SmartDashboard.putNumber("amp angle", Field.AMPLIFIER_SHOOTER_ANGLE);
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
    if (!intake.isNoteHeld()) {
      // intake.intakeYes(IntakeConstants.INTAKE_SPEED); // only code that runs the intake
      if(runsEmpty<21) {runsEmpty++;}
      if(runsEmpty>20) {
        if(humanPlayerSupplier.getAsBoolean()) {
          m_Indexer.set(IndexerConstants.HUMAN_PLAYER_INDEXER_SPEED);
          shooter.shootSameRPS(ShooterConstants.HUMAN_PLAYER_RPS);
          intake.intakeOff();
        }
        else {
          if(groundIntakeSup.getAsBoolean()) {
            m_Indexer.set(IndexerConstants.INDEXER_INTAKE_SPEED);
            intake.intakeYes(IntakeConstants.INTAKE_SPEED);
          } else {
            m_Indexer.off();
          }
          shooter.turnOff();
        }
      }
    } else {
      runsEmpty = 0;
      intake.intakeOff();
      if(revUpSupplier.getAsBoolean()||stageAngleSup.getAsBoolean()||subwooferAngleSup.getAsBoolean()) {
        if(angleShooterSubsytem.shortSpeakerDist()) {
          shooter.shootRPS(ShooterConstants.SHORT_SHOOTING_RPS);
        } else {
          shooter.shootRPS(ShooterConstants.LONG_SHOOTING_RPS);
        }
      } else {
        if (operatorRevSup.getAsBoolean()){ 
          shooter.shootRPSWithCurrent(100, 10, 20);
        } else {
          shooter.turnOff();
        }
      }
      boolean indexer = false;
      if(angleShooterSubsytem.validShot() && drivetrain.validShot() && shooter.validShot() && shooter.isReving()) {
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
      if((stageAngleSup.getAsBoolean()||subwooferAngleSup.getAsBoolean()||farStageAngleSup.getAsBoolean())&&revUpSupplier.getAsBoolean()&& shooter.validShot()) {
        indexer = true;
      }
      if (indexer) {
          m_Indexer.set(IndexerConstants.INDEXER_SHOOTING_SPEED);
          if(!intake.isNoteSeen()) {
            intake.setNoteHeld(false);
          }
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
