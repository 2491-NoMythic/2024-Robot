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
  BooleanSupplier intakeReverse; 
  BooleanSupplier OverStagePassSup; 
  boolean auto;
  double runsEmpty = 0;

  /**
   * A command to manage the control of notes throughout the robot. This command controls the Intake, Shooter, and Indexer. If any of these preferences are turned off, the command should not be initialized. If 
   * any of these subsytems are required in a different command, this command will not run during that command. This command controls: ground intake, rev'ing up the shooter if we are at a setpoint or auto-aiming, shooting if the shoot-if-ready button is pressed
   * and all the subsytems think we have a good shot, and collecting notes from the source.
   * <p>
   * the buttons fed into this command that match the names of buttons fed into the AimRobotMoving command and the AimShooter command should match.
   * @param m_IndexerSubsystem indexer subsystem
   * @param shootIfReadySupplier button to press to shoot if all subsystems think we have a good shot
   * @param revUpSupplier same as the auto-aim supplier that starts the aimRobotMoving command
   * @param shooter shooter subsystem
   * @param intake intake subsystem
   * @param drivetrain drivetrian subsystem
   * @param angleShooterSubsystem angle shooter subsytem
   * @param humanPlaySupplier button to press when intaking from source
   * @param stageAngleSup button to press to aim at the speaker from the podium
   * @param SubwooferSup button to press to aim at the speaker from the subwoofer
   * @param groundIntakeSup button to press to do ground intake
   * @param farStageAngleSup button to press to aim at the speaker from the far stage leg
   * @param operatorRevSup button to press to rev up the shooter slowly while driving
   * @param intakeReverse button to press to run the indexer backwards manually
   */
  public IndexCommand(IndexerSubsystem m_IndexerSubsystem, BooleanSupplier shootIfReadySupplier, BooleanSupplier revUpSupplier, ShooterSubsystem shooter, IntakeSubsystem intake, DrivetrainSubsystem drivetrain, AngleShooterSubsystem angleShooterSubsystem, BooleanSupplier humanPlaySupplier, BooleanSupplier stageAngleSup, BooleanSupplier SubwooferSup, BooleanSupplier groundIntakeSup, BooleanSupplier farStageAngleSup, BooleanSupplier operatorRevSup, BooleanSupplier intakeReverse, BooleanSupplier OverStagePassSup) {
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
    this.intakeReverse = intakeReverse;
    this.OverStagePassSup = OverStagePassSup;
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
    if (!intake.isNoteSeen()) {
      // intake.intakeYes(IntakeConstants.INTAKE_SPEED); // only code that runs the intake
      if(runsEmpty<21) {runsEmpty++;}
      if(runsEmpty>=20) {
        intake.setNoteHeld(false);
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
      if(revUpSupplier.getAsBoolean()||stageAngleSup.getAsBoolean()||subwooferAngleSup.getAsBoolean()||OverStagePassSup.getAsBoolean()) {
        if(OverStagePassSup.getAsBoolean()) {
          shooter.shootRPS(ShooterConstants.PASS_RPS);
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
      if((stageAngleSup.getAsBoolean()||subwooferAngleSup.getAsBoolean()||farStageAngleSup.getAsBoolean()||OverStagePassSup.getAsBoolean())&&shooter.isReving()&&revUpSupplier.getAsBoolean()&& shooter.validShot()&&shootIfReadySupplier.getAsBoolean()) {
        indexer = true;
      }
      if (indexer) {
          m_Indexer.set(IndexerConstants.INDEXER_SHOOTING_POWER);
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
