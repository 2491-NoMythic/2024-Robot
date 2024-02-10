package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    CANSparkMax m_IndexerMotor;
    SparkAnalogSensor m_DistanceSensor;

    public IndexerSubsystem() {
        m_IndexerMotor = new CANSparkMax(IndexerConstants.INDEXER_MOTOR, MotorType.kBrushless);
        m_DistanceSensor = m_IndexerMotor.getAnalog(Mode.kAbsolute);
    }
    
    public boolean isNoteIn() {
        return m_DistanceSensor.getVoltage()>2;
    }

    public void on() {
        m_IndexerMotor.set(IndexerConstants.INDEXER_SPEED);
    }

    public void off() {
        m_IndexerMotor.set(0);
    }

    public void reverse() {
        m_IndexerMotor.set(-IndexerConstants.INDEXER_SPEED);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("voltage sensor output", m_DistanceSensor.getVoltage());
    }
}
