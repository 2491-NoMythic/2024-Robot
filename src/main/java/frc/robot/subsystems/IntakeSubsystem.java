// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake1;
  SparkPIDController intake1Controller;
  CANSparkMax intake2;
  SparkPIDController intake2Controller;
  CANSparkMax intakeSideLeft;
  CANSparkMax intakeSideRight;
  SparkAnalogSensor m_DistanceSensor;
  boolean isNoteHeld;

  MotorLogger motorLogger1;
  MotorLogger motorLogger2;
  DoubleLogEntry logDistance;
  BooleanLogEntry logNoteIn;

  double intakeRunSpeed;

  public IntakeSubsystem() {
    intake1 = new CANSparkMax(IntakeConstants.INTAKE_1_MOTOR, MotorType.kBrushless);
    intake2 = new CANSparkMax(IntakeConstants.INTAKE_2_MOTOR, MotorType.kBrushless);
    intake1.restoreFactoryDefaults();
    intake2.restoreFactoryDefaults();
    intake1Controller = intake1.getPIDController();
    intake2Controller = intake2.getPIDController();
      intake1Controller.setP(IntakeConstants.INTAKE_1_kP);
      intake1Controller.setI(IntakeConstants.INTAKE_1_kI);
      intake1Controller.setD(IntakeConstants.INTAKE_1_kD);
      intake1Controller.setFF(IntakeConstants.INTAKE_1_kFF);
      intake2Controller.setP(IntakeConstants.INTAKE_2_kP);
      intake2Controller.setI(IntakeConstants.INTAKE_2_kI);
      intake2Controller.setD(IntakeConstants.INTAKE_2_kD);
      intake2Controller.setFF(IntakeConstants.INTAKE_2_kF);
    intake1.setInverted(true);
    intake2.setInverted(true);
    intake1.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake1.setSmartCurrentLimit(25, 40, 1000);
    intake2.setSmartCurrentLimit(25, 40, 1000);
    intake1.burnFlash();
    intake2.burnFlash();

    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft = new CANSparkMax(IntakeConstants.INTAKE_SIDE_MOTOR_LEFT, MotorType.kBrushless);
      intakeSideRight = new CANSparkMax(IntakeConstants.INTAKE_SIDE_MOTOR_RIGHT, MotorType.kBrushless);
      intakeSideLeft.restoreFactoryDefaults();
      intakeSideRight.restoreFactoryDefaults();
      intakeSideLeft.setInverted(false);
      intakeSideRight.setInverted(true);
      intakeSideLeft.setIdleMode(IdleMode.kCoast);
      intakeSideRight.setIdleMode(IdleMode.kCoast);
      intakeSideLeft.setSmartCurrentLimit(25, 40, 1000);
      intakeSideRight.setSmartCurrentLimit(25, 40, 1000);
      intakeSideRight.getEncoder().setPositionConversionFactor(1);
      intakeSideLeft.getEncoder().setPositionConversionFactor(1);
      intakeSideLeft.burnFlash();
      intakeSideRight.burnFlash();
      //intakeSideRight.getPIDController().setReference(intakeRunSpeed, CANSparkMax.ControlType.kVelocity)
    }
    m_DistanceSensor = intakeSideLeft.getAnalog(Mode.kAbsolute);

    DataLog log = DataLogManager.getLog();
    motorLogger1 = new MotorLogger(log, "/intake/motor1");
    motorLogger2 = new MotorLogger(log, "/intake/motor2");
    logDistance = new DoubleLogEntry(log, "/intake/noteDistance");
    logNoteIn = new BooleanLogEntry(log, "/intake/noteIn");
  }

  /**
   * sets the intakes speed
   * <p>
   * uses percentage of full power
   * 
   * @param intakeRunSpeed percentage of full power, from -1 to 1
   * @param intakeSideRunSpeed percentage of full power of the side wheels, from -1 to 1
   */
  public void intakeYes(double intakeRunSpeed, double intakeSideRunSpeed) {
    intake1.set(intakeRunSpeed);
    intake2.set(intakeRunSpeed);
    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft.set(intakeSideRunSpeed);
      intakeSideRight.set(intakeSideRunSpeed);
    }
  }

  public void setVelocity(double velocity) {
    intake1Controller.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    intake2Controller.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void intakeSideWheels(double sideWheelRunSpeed) {
    intakeSideLeft.set(sideWheelRunSpeed);
    intakeSideRight.set(sideWheelRunSpeed);
  }

  /**
   * sets the intakes speed
   * <p>
   * uses percentage of full power
   * 
   * @param intakeRunSpeed NEGATIVE percentage of full power
   */
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
    intake2.set(-intakeRunSpeed);
    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft.set(-intakeRunSpeed);
      intakeSideRight.set(-intakeRunSpeed);
    }
  }

  /**
   * sets the intake's power to 0
   */
  public void intakeOff() {
    intake1.set(0);
    intake2.set(0);
    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft.set(0);
      intakeSideRight.set(0);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("voltage sensor output", m_DistanceSensor.getVoltage());
    SmartDashboard.putNumber("INTAKE/leading roller speed", intake1.getEncoder().getVelocity());
    SmartDashboard.putNumber("INTAKE/trailing roller speed", intake2.getEncoder().getVelocity());
    motorLogger1.log(intake1);
    motorLogger2.log(intake2);
    logDistance.append(m_DistanceSensor.getVoltage());
    logNoteIn.append(RobotState.getInstance().isNoteSeen());
    RobotState.getInstance().intakeSensorVoltage = m_DistanceSensor.getVoltage();
  }
}
