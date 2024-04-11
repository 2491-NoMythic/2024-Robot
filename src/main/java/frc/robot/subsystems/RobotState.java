package frc.robot.subsystems;

public class RobotState {
	private static RobotState instance;
	public boolean IsNoteSeen;
	public boolean IsNoteHeld;
	public boolean ShooterInRange;
	public double ShooterError;
	public boolean LimelightsUpdated;
	public boolean lightsReset;

	private RobotState() {

	}

	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}
}
