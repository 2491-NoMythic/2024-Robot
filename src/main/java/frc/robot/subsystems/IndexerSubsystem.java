package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.SparkAnalogSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    TalonFX indexerMotor;
    SparkAnalogSensor distanceSensor;
    CurrentLimitsConfigs currentLimitsConfigs;

    public IndexerSubsystem() {

        indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR);

        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = IndexerConstants.CURRENT_LIMIT;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
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
        indexerMotor.getConfigurator().apply(talonFXConfig);

    }
    /**
     * sets the speed of the indexer motor to INDEXER_INTAKE_SPEED (from constants)
     * <p>
     * uses percentage of full power
     */
    public void on() {
        indexerMotor.set(IndexerConstants.INDEXER_INTAKE_SPEED);
    }
    /**
    sets the indxer motor's percent-of-full-power to 0
     */
    public void off() {
        indexerMotor.set(0);
    }
    /**
     * sets the indexer motor to -INDEXER_INTAKE_SPEED (from constants)
     * <p>
     * uses percentage of full power
     */
    public void reverse() {
        indexerMotor.set(-IndexerConstants.INDEXER_INTAKE_SPEED);
    }
    /**
     * sets the percentage-of-full-power on the indexer
     * @param speed the desired speed, from -1 to 1
     */
    public void set(double speed) {
        indexerMotor.set(speed);
    }
    /**
     * uses the indexer motor's onboard Motion Magic control to move the indexer forward. To move backwards, use negative inches.
     * @param inches the inches to move forward.
     */
    public void forwardInches(double inches) {
        double rotationsRequested = inches/IndexerConstants.MOTOR_ROTATIONS_TO_INCHES;
        double position = indexerMotor.getPosition().getValueAsDouble()+rotationsRequested;
        MotionMagicVoltage distanceRequest = new MotionMagicVoltage(position);
        indexerMotor.setControl(distanceRequest);
    }
    /**
     * uses the indexer motor's onboard Motion Magic control to set the motor to a desired RPS
     * @param RPS the desired speed, in rotations per second
     */
    public void magicRPS(double RPS) {
        MotionMagicVelocityVoltage speedRequest = new MotionMagicVelocityVoltage(RPS);
        indexerMotor.setControl(speedRequest);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("indexer current", indexerMotor.getSupplyCurrent().getValueAsDouble());
    }
}
