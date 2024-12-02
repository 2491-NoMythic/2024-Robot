// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.subsystems.AngleShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotState;

public class GroundIntake extends Command {
  /** Creates a new GroundIntake. */
  IntakeSubsystem intake;
  IndexerSubsystem indexer;
  DrivetrainSubsystem driveTrain;
  AngleShooterSubsystem angleShooter;
  public GroundIntake(IntakeSubsystem intake, IndexerSubsystem indexer, DrivetrainSubsystem driveTrain, AngleShooterSubsystem angleShooter) {
    addRequirements(intake, indexer, angleShooter);
    this.intake = intake;
    this.indexer = indexer;
    this.driveTrain = driveTrain;
    this.angleShooter = angleShooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotState.getInstance().isNoteSeen()){
      RobotState.getInstance().lightsReset = true;
    }
    angleShooter.setDesiredShooterAngle(30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotState.getInstance().lightsReset = false;
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
    // indexer.magicRPS(0);
    if(DriverStation.isAutonomous()) {
      // indexer.forwardInches(-1);
      indexer.off();
    } else {
      indexer.off();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotState.getInstance().isNoteSeen();
  }
}
