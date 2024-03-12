// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveMeters extends Command {
  /** Creates a new MoveMeters. */
  DrivetrainSubsystem m_drivetrain;
  double m_meters;
  double m_forwardSpeed;
  double m_rightSpeed;
  double m_angleSpeed;
  double startY;
  double startX;
  double whileX;
  double whileY;
  double distance;
  double distanceX;
  double distanceY;
  Pose2d pose;
  public MoveMeters(DrivetrainSubsystem drivetrain, double meters, double forwardSpeed, double rightSpeed, double angleSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_meters = meters;
    m_forwardSpeed = forwardSpeed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pose = m_drivetrain.getPose();
    startX = pose.getX();
    startY = pose.getY();
    m_drivetrain.drive(new ChassisSpeeds(m_forwardSpeed, m_rightSpeed, m_angleSpeed));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pose = m_drivetrain.getPose();
    whileX = pose.getX();
    whileY = pose.getY();
    distanceX = (startX - whileX);
    distanceY = (startY - whileY);
    distance = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance >= m_meters;
  }
}
