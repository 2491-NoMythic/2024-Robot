
package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

public class SprkDynmcCmtLimit {
	private static enum PowerState {
		UNKNOWN,
		LOW,
		HIGH,
	}

	private double safeWatts;
	private double limitLow;
	private double limitHigh;
	private double currentEnergy;
	private double lastUpdate;
	private PowerState powerState;
	private int currentHigh;
	private int currentLow;

	public SprkDynmcCmtLimit(double safeWatts, double limitLow, double limitHigh, int currentLow, int currentHigh) {
		this.safeWatts = safeWatts;
		this.limitLow = limitLow;
		this.limitHigh = limitHigh;
		this.currentEnergy = limitHigh;
		this.lastUpdate = Timer.getFPGATimestamp();
		this.powerState = PowerState.UNKNOWN;
		this.currentHigh = currentHigh;
		this.currentLow = currentLow;
	}

	public void Update(CANSparkMax motor) {
		double volts = motor.getBusVoltage() * motor.getAppliedOutput();
		double amps = motor.getOutputCurrent();
		double watts = volts * amps;
		double now = Timer.getFPGATimestamp();
		double elapsed = now - lastUpdate;
		lastUpdate = now;
		double joules = watts * elapsed;
		double safejoules = safeWatts * elapsed;

		currentEnergy += joules - safejoules;
		if (currentEnergy <= 0) {
			currentEnergy = 0;
		}

		PowerState newState;
		if (powerState == PowerState.HIGH) {
			if (currentEnergy > limitHigh) {
				newState = PowerState.LOW;
			} else {
				newState = PowerState.HIGH;
			}
			} else {
			if (currentEnergy <= limitLow) {
				newState = PowerState.HIGH;
			} else {
				newState = PowerState.LOW;
			}
		}

		if(newState != powerState){
			motor.setSmartCurrentLimit(newState == PowerState.HIGH ? currentHigh : currentLow);
			powerState = newState;
		}
	}
}
