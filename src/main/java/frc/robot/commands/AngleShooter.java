// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AngleShooter extends Command {
  /** Creates a new AngleShooter. */
  double desiredShooterAngle;
  double currentShooterAngle;
  double differenceAngle;
  double angleSpeed;
  ShooterSubsystem m_shooter;
  double desiredShooterAngleSpeed;

  
  public AngleShooter(ShooterSubsystem shooter, double desiredShooterAngle) {
    // Use addRequirements() here to declare subsystem dependencies.\
    m_shooter = shooter;
    this.desiredShooterAngle = desiredShooterAngle;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    differenceAngle = desiredShooterAngle-m_shooter.getShooterAngle;
    desiredShooterAngleSpeed = differenceAngle*ShooterConstants.AUTO_AIM_SHOOTER_kP;
    m_shooter.pitchShooter(desiredShooterAngleSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.pitchShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
