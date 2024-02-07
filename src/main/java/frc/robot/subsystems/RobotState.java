package frc.robot.subsystems;

public class RobotState {
	private static RobotState instance;
	public boolean IsNoteSeen;
	public boolean ShooterInRange;
	public boolean ShooterReady;

	private RobotState() {

	}

	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}
}
