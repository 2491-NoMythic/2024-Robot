package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;

public class AimShooter extends Command {
	AngleShooterSubsystem angleShooterSubsystem;
	BooleanSupplier aimAtAmp;

	public AimShooter(AngleShooterSubsystem angleShooterSubsystem, BooleanSupplier aimAtAmp) {
		this.angleShooterSubsystem = angleShooterSubsystem;
		this.aimAtAmp = aimAtAmp;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (!aimAtAmp.getAsBoolean()) {
			double desiredShooterAngleSpeed = angleShooterSubsystem.calculateSpeakerAngleDifference() * ShooterConstants.AUTO_AIM_SHOOTER_kP;
			angleShooterSubsystem.pitchShooter(desiredShooterAngleSpeed);
		} else {
			double differenceAmp = Field.AMPLIFIER_ANGLE - angleShooterSubsystem.getShooterAngle();
			double ampSpeed = differenceAmp * ShooterConstants.AUTO_AIM_SHOOTER_kP;
			angleShooterSubsystem.pitchShooter(ampSpeed);
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
