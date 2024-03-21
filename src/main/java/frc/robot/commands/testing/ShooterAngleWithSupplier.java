// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterAngleWithSupplier extends Command {
  /** Creates a new ShooterSpeedWithSupplier. */
  DoubleSupplier angleSup;
  AngleShooterSubsystem angleShooter;
  public ShooterAngleWithSupplier(DoubleSupplier angleSup, AngleShooterSubsystem angleShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angleShooter);
    this.angleShooter = angleShooter;
    this.angleSup = angleSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleShooter.setDesiredShooterAngle(angleSup.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleShooter.setDesiredShooterAngle(angleShooter.calculateSpeakerAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
