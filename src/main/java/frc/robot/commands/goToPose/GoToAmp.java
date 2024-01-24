// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.goToPose;

import static frc.robot.settings.Constants.DriveConstants.DEFAUL_PATH_CONSTRAINTS;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.PathConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GoToAmp extends Command {
  Command actualCommand;
  /** Creates a new GoToAmp. */
  BooleanSupplier isAllianceRed;
  DrivetrainSubsystem driveTrain;
  public GoToAmp(DrivetrainSubsystem drivetrain, BooleanSupplier isAllianceRedSupl) {
    isAllianceRed = isAllianceRedSupl;
    this.driveTrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isAllianceRed.getAsBoolean()) {
      actualCommand = AutoBuilder.pathfindToPose(PathConstants.AMP_RED_POSE, DEFAUL_PATH_CONSTRAINTS);
    }
    else{
      actualCommand = AutoBuilder.pathfindToPose(PathConstants.AMP_BLUE_POSE, DEFAUL_PATH_CONSTRAINTS);
    }
    actualCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    actualCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return actualCommand.isFinished();
  }
}
