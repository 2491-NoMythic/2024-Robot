package frc.robot.subsystems;

public class RobotState {
	private static RobotState instance;
	public boolean IsNoteSeen;
	public boolean IsNoteHeld;
	public boolean ShooterInRange;
	public boolean ShooterReady;
	public double ShooterError;
	public double ShooterVelocity;
	public boolean LimelightsUpdated;

	private RobotState() {

	}

	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}
}
