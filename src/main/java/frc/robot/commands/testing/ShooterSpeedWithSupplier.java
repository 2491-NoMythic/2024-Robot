// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterSpeedWithSupplier extends Command {
  /** Creates a new ShooterSpeedWithSupplier. */
  DoubleSupplier speedSup;
  ShooterSubsystem shooter;
  BooleanSupplier sameSpeedSup;
  public ShooterSpeedWithSupplier(DoubleSupplier speedSup, ShooterSubsystem shooter, BooleanSupplier sameSpeedSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
    this.speedSup = speedSup;
    this.sameSpeedSup = sameSpeedSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sameSpeedSup.getAsBoolean()) {
      shooter.shootSameRPS(speedSup.getAsDouble());
    } else {
      shooter.shootRPS(speedSup.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
