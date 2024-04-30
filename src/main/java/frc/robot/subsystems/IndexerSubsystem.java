package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
        m_IndexerMotor.setNeutralMode(NeutralModeValue.Brake);
        this.isNoteIn = isNoteIn;

        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = IndexerConstants.CURRENT_LIMIT;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        configurator = m_IndexerMotor.getConfigurator();
        configurator.apply(currentLimitsConfigs);

        motorLogger = new MotorLogger(DataLogManager.getLog(), "/indexer/motor");
        notePositionLog = new DoubleLogEntry(DataLogManager.getLog(),"/indexer/notePosistion");
    
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration().withCurrentLimits(currentLimitsConfigs)
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs().
                    withKS(IndexerConstants.INDEXER_KS).
                    withKV(IndexerConstants.INDEXER_KV).
                    withKA(IndexerConstants.INDEXER_KA).
                    withKP(IndexerConstants.INDEXER_KP))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(IndexerConstants.INDEXER_CRUISE_VELOCITY)
                        .withMotionMagicAcceleration(IndexerConstants.INDEXER_ACCELERATION)
                        .withMotionMagicJerk(IndexerConstants.INDEXER_JERK));
        m_IndexerMotor.getConfigurator().apply(talonFXConfig);

    }
    /**
     * sets the speed of the indexer motor to INDEXER_INTAKE_SPEED (from constants)
     * <p>
     * uses percentage of full power
     */
    public void on() {
        m_IndexerMotor.set(IndexerConstants.INDEXER_INTAKE_SPEED);
    }
    /**
    sets the indxer motor's percent-of-full-power to 0
     */
    public void off() {
        m_IndexerMotor.set(0);
    }
    /**
     * sets the indexer motor to -INDEXER_INTAKE_SPEED (from constants)
     * <p>
     * uses percentage of full power
     */
    public void reverse() {
        m_IndexerMotor.set(-IndexerConstants.INDEXER_INTAKE_SPEED);
    }
    /**
     * sets the percentage-of-full-power on the indexer
     * @param speed the desired speed, from -1 to 1
     */
    public void set(double speed) {
        m_IndexerMotor.set(speed);
    }
    public void setVoltage(double voltage) {
        m_IndexerMotor.setControl(new VoltageOut(voltage));
    }
    /**
     * uses the indexer motor's onboard Motion Magic control to move the indexer forward. To move backwards, use negative inches.
     * @param inches the inches to move forward.
     */
    public void forwardInches(double inches) {
        double rotationsRequested = inches/IndexerConstants.MOTOR_ROTATIONS_TO_INCHES;
        double position = m_IndexerMotor.getPosition().getValueAsDouble()+rotationsRequested;
        MotionMagicVoltage distanceRequest = new MotionMagicVoltage(position);
        m_IndexerMotor.setControl(distanceRequest);
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
    /**
     * uses the indexer motor's onboard Motion Magic control to set the motor to a desired RPS
     * @param RPS the desired speed, in rotations per second
     */
    public void magicRPS(double RPS) {
        MotionMagicVelocityVoltage speedRequest = new MotionMagicVelocityVoltage(RPS);
        m_IndexerMotor.setControl(speedRequest);
    }

    public void magicRPSSupplier(DoubleSupplier RPSSupplier) {
        MotionMagicVelocityVoltage speedRequest = new MotionMagicVelocityVoltage(RPSSupplier.getAsDouble());
        m_IndexerMotor.setControl(speedRequest);
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
