// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooterSubsystem;

public class AngleShooter extends Command {
  /** Creates a new AngleShooter. */
  double desiredShooterAngle;
  DoubleSupplier desiredShooterAngleSupplier;
  double currentShooterAngle;
  double differenceAngle;
  double angleSpeed;
  AngleShooterSubsystem m_shooter;
  double desiredShooterAngleSpeed;
  
  
  public AngleShooter(AngleShooterSubsystem shooter, DoubleSupplier desiredShooterAngleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.\
    m_shooter = shooter;
    this.desiredShooterAngleSupplier = desiredShooterAngleSupplier;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredShooterAngle = desiredShooterAngleSupplier.getAsDouble();
    m_shooter.setDesiredShooterAngle(desiredShooterAngle);
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
