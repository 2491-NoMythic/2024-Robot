package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkAnalogSensor;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    TalonFX m_IndexerMotor;
    CurrentLimitsConfigs currentLimitsConfigs;
    TalonFXConfigurator configurator;
    BooleanSupplier isNoteIn;
    double noteStart;
    boolean wasNoteIn = false;
    MotorLogger motorLogger;
    DoubleLogEntry notePositionLog;

    public IndexerSubsystem(BooleanSupplier isNoteIn) {
        m_IndexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR);
        m_IndexerMotor.setInverted(false);
        this.isNoteIn = isNoteIn;

        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = IndexerConstants.CURRENT_LIMIT;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        configurator = m_IndexerMotor.getConfigurator();
        configurator.apply(currentLimitsConfigs);

        motorLogger = new MotorLogger(DataLogManager.getLog(), "/indexer/motor");
        notePositionLog = new DoubleLogEntry(DataLogManager.getLog(),"/indexer/notePosistion");
    }

    public void on() {
        m_IndexerMotor.set(IndexerConstants.INDEXER_INTAKE_SPEED);
    }

    public void off() {
        m_IndexerMotor.set(0);
    }

    public void reverse() {
        m_IndexerMotor.set(-IndexerConstants.INDEXER_INTAKE_SPEED);
    }
    public void set(double speed) {
        m_IndexerMotor.set(speed);
    }

    public void trackNote() {
        boolean noteIn = isNoteIn.getAsBoolean();
        if (!wasNoteIn && noteIn) {
            noteStart = m_IndexerMotor.getPosition().getValueAsDouble();
        }
        wasNoteIn = noteIn;
    }

    public double getNotePosition() {
        return m_IndexerMotor.getPosition().getValueAsDouble() - noteStart;
    }

    @Override
    public void periodic() {
        trackNote();
        motorLogger.log(m_IndexerMotor);
        double pos = getNotePosition();
        notePositionLog.append(pos);
        SmartDashboard.putNumber("note pos", pos);
    }
}
