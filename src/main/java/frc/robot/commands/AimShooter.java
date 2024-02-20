package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;

public class AimShooter extends Command {
	AngleShooterSubsystem angleShooterSubsystem;
	DoubleSupplier aimAtAmpSupplier;

	public AimShooter(AngleShooterSubsystem angleShooterSubsystem, DoubleSupplier aimAtAmp) {
		addRequirements(angleShooterSubsystem); 
		this.angleShooterSubsystem = angleShooterSubsystem;
		this.aimAtAmpSupplier = aimAtAmp;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (aimAtAmpSupplier.getAsDouble() == 90) {
			angleShooterSubsystem.setDesiredShooterAngle(SmartDashboard.getNumber("amp angle", Field.AMPLIFIER_ANGLE)/*Field.AMPLIFIER_ANGLE*/);
		} else {
			angleShooterSubsystem.setDesiredShooterAngle(angleShooterSubsystem.calculateSpeakerAngle());
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		angleShooterSubsystem.pitchShooter(0);
	}
}
