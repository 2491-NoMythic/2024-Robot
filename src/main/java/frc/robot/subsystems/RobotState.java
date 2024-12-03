package frc.robot.subsystems;

public class RobotState {
	private static RobotState instance;
	public boolean LimelightsUpdated;
	public boolean lightsReset;
	public double odometerOrientation;

	private RobotState() {

	}
	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}
}
