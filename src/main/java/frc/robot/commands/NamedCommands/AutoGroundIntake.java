// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

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
public class AutoGroundIntake extends Command {
  /** Creates a new ConditionalIndexer. */
  IndexerSubsystem indexer;
  IntakeSubsystem intake;
  AngleShooterSubsystem angleShooterSubsystem;
  DrivetrainSubsystem driveTrain;
  public AutoGroundIntake(IndexerSubsystem indexer, IntakeSubsystem intake, DrivetrainSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.\
   addRequirements(indexer,intake);
    this.indexer = indexer;
    this.intake = intake;
    this.driveTrain = driveTrain;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mult = 0.43;
    double robotSpeed = Math.sqrt(Math.pow(driveTrain.getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(driveTrain.getChassisSpeeds().vyMetersPerSecond, 2));
    double rollerSpeed = (IntakeConstants.INTAKE_SPEED*IntakeConstants.INTAKE_MAX_VELOCITY - (mult * IntakeConstants.INTAKE_SPEED*IntakeConstants.INTAKE_MAX_VELOCITY)) * (robotSpeed / /*DriveConstants.MAX_VELOCITY_METERS_PER_SECOND*/2.4) + (mult * IntakeConstants.INTAKE_SPEED*IntakeConstants.INTAKE_MAX_VELOCITY);
    // double rollerSpeed = ( - (mult * 9000)) * (robotSpeed / /*DriveConstants.MAX_VELOCITY_METERS_PER_SECOND*/2.4) + (mult * 10000);
    double sideSpeed =  (IntakeConstants.INTAKE_SIDE_SPEED - (mult * IntakeConstants.INTAKE_SIDE_SPEED)) * (robotSpeed / /*DriveConstants.MAX_VELOCITY_METERS_PER_SECOND*/2.4) + (mult * IntakeConstants.INTAKE_SIDE_SPEED);
    double indexerSpeed = (IndexerConstants.INDEXER_INTAKE_SPEED- (mult * IndexerConstants.INDEXER_INTAKE_SPEED)) * (robotSpeed / /*DriveConstants.MAX_VELOCITY_METERS_PER_SECOND*/2.4) + (mult * IndexerConstants.INDEXER_INTAKE_SPEED);
    // intake.intakeYes(rollerSpeed, sideSpeed);
    intake.setVelocity(rollerSpeed);
    SmartDashboard.putNumber("GROUNDINTAKE/roller speed", rollerSpeed);
    intake.intakeSideWheels(sideSpeed);
    indexer.set(indexerSpeed);
    // intake.intakeYes(0.6, 0.6);
    // indexer.set(0.9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    indexer.off();
    RobotState.getInstance().IsNoteHeld = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotState.getInstance().isNoteSeen();
  }
}
