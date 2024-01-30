package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    CANSparkMax m_IndexerMotor;
    boolean Indexer;

    public IndexerSubsystem() {
        CANSparkMax indexerMotor = new CANSparkMax(IndexerConstants.INDEXER_MOTOR, MotorType.kBrushless);
        indexerMotor = m_IndexerMotor;
    }

    public double getInchesFromSensor() {
        return 0;
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
}
