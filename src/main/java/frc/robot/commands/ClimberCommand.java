// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  Climber climber;
  DoubleSupplier translationYSupplierLeft;
  DoubleSupplier translationYSupplierRight;
/*
 * moves the climbers at a specific speed
 */
  public ClimberCommand(Climber climber, DoubleSupplier translationYSupplierLeft, DoubleSupplier translationYSupplierRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.translationYSupplierLeft = translationYSupplierLeft;
    this.translationYSupplierRight = translationYSupplierRight;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climberGoLeft(translationYSupplierLeft.getAsDouble());
    climber.climberGoRight(translationYSupplierRight.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
