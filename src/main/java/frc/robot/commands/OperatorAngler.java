// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;

public class OperatorAngler extends Command {
	AngleShooterSubsystem angler;
	PS4Controller opController;
	/** Creates a new OperatorShooter. */
	public OperatorAngler(AngleShooterSubsystem angleShooterSubsystem, PS4Controller operatorController) {
		// Use addRequirements() here to declare subsystem dependencies.
		angler = angleShooterSubsystem;
		opController = operatorController;
		addRequirements(angleShooterSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		angler.getDesiredShooterAngle();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (opController.getPOV() == 90){
			angler.setDesiredShooterAngle(ShooterConstants.MINIMUM_SHOOTER_ANGLE);
		} else if (opController.getPOV() == 270) {
			angler.setDesiredShooterAngle(ShooterConstants.HUMAN_PLAYER_ANGLE);
		} else if (opController.getCircleButton()) {
			angler.setDesiredShooterAngle(45);
		
		} else {
			angler.setDesiredShooterAngle(angler.getDesiredShooterAngle() + opController.getLeftY()*0.2);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
