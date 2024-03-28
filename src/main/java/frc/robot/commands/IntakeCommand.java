// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.settings.IntakeDirection;
import frc.robot.settings.Constants.IntakeConstants;
public class IntakeCommand extends Command {
  /** Creates a new IntakeLeftCommand. */
  IntakeSubsystem intake;
  IntakeDirection iDirection;
  public IntakeCommand(IntakeSubsystem intake, IntakeDirection iDirection) {
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.iDirection = iDirection;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (iDirection) {
      case COAST:
        intake.intakeOff();
        break;
      case INTAKE:
        intake.intakeYes(IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SIDE_SPEED);
        break;
      case OUTAKE:
        intake.intakeNo(IntakeConstants.INTAKE_SPEED);
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
