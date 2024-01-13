// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_PIGEON_ID;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAim extends Command {
  /** Creates a new autoAim. */
  private final PS4Controller m_AutoAimButton;
  DrivetrainSubsystem m_drivetrain;
  ShooterSubsystem m_ShooterSubsystem;
  double m_desiredRobotAngle;
  double differenceAngle;
  double currentHeading;
  double speakerDist;
  double speakerA;
  double speakerB;
  double m_DesiredShooterAngle;
  double turningSpeed;
  RotateRobot rotateRobot;
  AngleShooter angleShooter;
  Pigeon2 pigeon2;

  public AutoAim( DrivetrainSubsystem drivetrain, ShooterSubsystem shooterSubsystem, PS4Controller autoAimButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    Pigeon2 pigeon2 = new Pigeon2(DRIVETRAIN_PIGEON_ID);
    m_drivetrain = drivetrain;
    m_ShooterSubsystem = shooterSubsystem;
    m_AutoAimButton = autoAimButton;

    addRequirements(drivetrain);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d dtvalues = m_drivetrain.getPose();
    currentHeading = pigeon2.getAngle();
    
    //triangle for robot angle
    speakerA = Math.abs(dtvalues.getX() - Field.SPEAKER_X);
    speakerB = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
    speakerDist = Math.sqrt(Math.pow(speakerA, 2) + Math.pow(speakerB, 2));

    //getting desired robot angle
    

    if (dtvalues.getY() <= Field.SPEAKER_Y) {
        double thetaBelow = (Math.asin(speakerA / speakerDist)) + 90;
        m_desiredRobotAngle = thetaBelow;
    }
    else{
        double thetaAbove = 360 - (Math.asin(speakerA / speakerDist) + 90);
        m_desiredRobotAngle = thetaAbove;
    }

    //triangle for shooter angle
    //TODO: Find out if we need projectile motion physics and implement it
    m_DesiredShooterAngle = Math.atan(Field.SPEAKER_Z / speakerDist);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_AutoAimButton.getCrossButton()){
    new ParallelCommandGroup(new RotateRobot(m_drivetrain, m_desiredRobotAngle), new AngleShooter(m_ShooterSubsystem, m_DesiredShooterAngle));
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(differenceAngle) < ShooterConstants.ROBOT_ANGLE_TOLERANCE && Math.abs(differenceAngle) < ShooterConstants.SHOOTER_ANGLE_TOLERANCE;
  }
}
