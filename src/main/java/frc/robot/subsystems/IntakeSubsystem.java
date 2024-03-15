// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 import com.revrobotics.CANSparkMax;
 import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
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
  CANSparkMax intake2;
  SparkAnalogSensor m_DistanceSensor;

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
    if(Preferences.getBoolean("CompBot", true)) {
      m_DistanceSensor = intake1.getAnalog(Mode.kAbsolute);
    } else {
      m_DistanceSensor = intake2.getAnalog(Mode.kAbsolute);
    }
    intake2.follow(intake1);
    intake2.setInverted(false);
    intake1.setInverted(true);
    intake1.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake1.setSmartCurrentLimit(25, 40, 1000);
    intake2.setSmartCurrentLimit(25, 40, 1000);
    intake1.burnFlash();
    intake2.burnFlash();

    DataLog log = DataLogManager.getLog();
    motorLogger1 = new MotorLogger(log, "/intake/motor1");
    motorLogger2 = new MotorLogger(log, "/intake/motor2");
    logDistance = new DoubleLogEntry(log, "/intake/noteDistance");
    logNoteIn = new BooleanLogEntry(log, "/intake/noteIn");
  }

  public void intakeYes(double intakeRunSpeed) {
    intake1.set(intakeRunSpeed);
  }
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
  }
  public void intakeOff() {
    intake1.set(0);
  }
  public boolean isNoteIn() {
    return m_DistanceSensor.getVoltage()<2;
  }
  @Override
  public void periodic() {
  SmartDashboard.putNumber("voltage sensor output", m_DistanceSensor.getVoltage());
  SmartDashboard.putBoolean("is note in", isNoteIn());
  
  motorLogger1.log(intake1);
  motorLogger2.log(intake2);
  logDistance.append(m_DistanceSensor.getVoltage());
  logNoteIn.append(isNoteIn());
  }
}
