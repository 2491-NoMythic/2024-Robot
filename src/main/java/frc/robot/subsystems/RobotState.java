package frc.robot.subsystems;

public class RobotState {
	private static RobotState instance;
	public boolean IsNoteSeen;
	public boolean IsNoteHeld;
	public boolean ShooterInRange;
	public double ShooterError;
	public boolean LimelightsUpdated;
	public boolean lightsReset;
	public double intakeSensorVoltage;
	public double odometerOrientation;

	private RobotState() {

	}
	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}

	public boolean isNoteSeen() {
		return intakeSensorVoltage < 2;
	}

	public void setNoteHeld(boolean held) {
		IsNoteHeld = held;
	}
}
