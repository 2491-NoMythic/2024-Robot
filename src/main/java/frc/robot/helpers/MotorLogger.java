package frc.robot.helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

/** 
 * logs moter data to a logger
 */
public class MotorLogger {
    DoubleLogEntry percentOutput;
    DoubleLogEntry current;
    DoubleLogEntry velocity;

    public MotorLogger(DataLog log, String path) {
        percentOutput = new DoubleLogEntry(log, path + "/output");
        current = new DoubleLogEntry(log, path + "/current");
        velocity = new DoubleLogEntry(log, path + "/velocity");
    }

    public void log(CANSparkMax motor) {
        current.append(motor.getOutputCurrent());
        percentOutput.append(motor.getAppliedOutput());
        velocity.append(motor.getEncoder().getVelocity()/60);
    }

    public void log(TalonFX motor) {
        current.append(motor.getStatorCurrent().getValueAsDouble());
        percentOutput.append(motor.get());
        velocity.append(motor.getVelocity().getValueAsDouble());
    }
}
