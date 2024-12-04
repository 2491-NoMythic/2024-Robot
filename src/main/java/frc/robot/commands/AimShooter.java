package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.AngleShooterSubsystem;

public class AimShooter extends Command {
	AngleShooterSubsystem angleShooterSubsystem;
	BooleanSupplier AmpSup;
	BooleanSupplier humanPlayerSupplier;
	BooleanSupplier SubwooferSupplier1;
	BooleanSupplier StageAngleSupplier;
	BooleanSupplier groundIntakeSup;
	BooleanSupplier farStageAngleSup;
	BooleanSupplier OverStageAngleSup;
	BooleanSupplier OppositeStageShotSup;
	BooleanSupplier LowPassAngleSup;
	/**
	 * A Command that will control the angle of the shooter. If none of the supplied buttons are pressed, than it will automatically aim at the speaker
	 * @param angleShooterSubsystem the subsytem to control the angle of the shooter
	 * @param POVSupplier a button that tells us the output of the POV buttons on the controller
	 * @param humanPlayerSupplier a button to set the shooter to the intake-from-source angle
	 * @param SubwooferSupplier1 a button to set the shooter to the correct speaker angle while the robot is against the subwoofer
	 * @param StageAngleSupplier a button to set the shooter to the correct speaker angle while the robot is against the podium
	 * @param groundIntakeSup a button to set the shooter to the correct angle for a succesful ground intake
	 * @param farStageAngleSup a button to set the shooter to the correct speaker angle while the robot is against the far stage leg
	 * @param OverStagePassSup button to press to aim at the speaker from the opposite side of the stage (from the speaker)
   	 * @param OppositeStageShotSup button to press to aim at the speaker from the opposite side of the stage (from the speaker)
	 */
	public AimShooter(AngleShooterSubsystem angleShooterSubsystem, BooleanSupplier AmpSup, BooleanSupplier humanPlayerSupplier,
					  BooleanSupplier SubwooferSupplier1, BooleanSupplier StageAngleSupplier, BooleanSupplier groundIntakeSup, BooleanSupplier farStageAngleSup, BooleanSupplier OverStageAngleSup, BooleanSupplier OppositeStageShotSup, BooleanSupplier LowPassAngleSup) {
		this.angleShooterSubsystem = angleShooterSubsystem;
		this.AmpSup = AmpSup;
		this.humanPlayerSupplier = humanPlayerSupplier;
		this.SubwooferSupplier1 = SubwooferSupplier1;
		this.StageAngleSupplier = StageAngleSupplier;
		this.groundIntakeSup = ()->false;
		this.farStageAngleSup = farStageAngleSup;
		this.OverStageAngleSup = OverStageAngleSup;
		this.OppositeStageShotSup = OppositeStageShotSup;
		this.LowPassAngleSup = LowPassAngleSup;
		addRequirements(angleShooterSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		angleShooterSubsystem.setDesiredShooterAngle(45);
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
