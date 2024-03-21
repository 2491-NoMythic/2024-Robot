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
    if(Preferences.getBoolean("CompBot", true)) {
      m_DistanceSensor = intake1.getAnalog(Mode.kAbsolute);
    } else {
      m_DistanceSensor = intake2.getAnalog(Mode.kAbsolute);
    }
    intake2.setInverted(true);
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
  /**
   * sets the intakes speed
   * <p> uses percentage of full power
   * @param intakeRunSpeed percentage of full power, from -1 to 1
   */
  public void intakeYes(double intakeRunSpeed) {
    intake1.set(intakeRunSpeed);
    intake2.set(intakeRunSpeed);
  }
  /**
   * sets the intakes speed
   * <p> uses percentage of full power
   * @param intakeRunSpeed NEGATIVE percentage of full power
   */
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
    intake2.set(-intakeRunSpeed);
  }
  /**
   * sets the intake's power to 0
   */
  public void intakeOff() {
    intake1.set(0);
    intake2.set(0);
  }
  /**
   * uses the distance sensor inside the indexer to tell if there is a note fully inside the indexer
   * @return if the sensor sees something within it's range in front of it
   */
  public boolean isNoteSeen() {
    return m_DistanceSensor.getVoltage()<2;
  }
  public boolean isNoteHeld() {
    return isNoteHeld;
  }
  public void setNoteHeld(boolean held) {
    isNoteHeld = held;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("voltage sensor output", m_DistanceSensor.getVoltage());  
    motorLogger1.log(intake1);
    motorLogger2.log(intake2);
    logDistance.append(m_DistanceSensor.getVoltage());
    logNoteIn.append(isNoteSeen());
    SmartDashboard.putBoolean("is note in", isNoteSeen());
    SmartDashboard.putBoolean("is note held", isNoteHeld());
  }
}
