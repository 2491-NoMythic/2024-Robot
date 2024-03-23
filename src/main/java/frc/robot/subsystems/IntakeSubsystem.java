// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 import com.revrobotics.CANSparkMax;
 import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.settings.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake1;
  CANSparkMax intake2;
  CANSparkMax intakeSideLeft;
  CANSparkMax intakeSideRight;
  SparkAnalogSensor m_DistanceSensor;
  boolean isNoteHeld;

  double intakeRunSpeed;
  public IntakeSubsystem() {
    intake1 = new CANSparkMax(IntakeConstants.INTAKE_1_MOTOR, MotorType.kBrushless);
    intake2 = new CANSparkMax(IntakeConstants.INTAKE_2_MOTOR, MotorType.kBrushless);
    intakeSideLeft = new CANSparkMax(IntakeConstants.INTAKE_SIDE_MOTOR_LEFT, MotorType.kBrushless);
    intakeSideRight = new CANSparkMax(IntakeConstants.INTAKE_SIDE_MOTOR_RIGHT, MotorType.kBrushless);
    intake1.restoreFactoryDefaults();
    intake2.restoreFactoryDefaults();
    intakeSideLeft.restoreFactoryDefaults();
    intakeSideRight.restoreFactoryDefaults();
    if(Preferences.getBoolean("CompBot", true)) {
      m_DistanceSensor = intake1.getAnalog(Mode.kAbsolute);
    } else {
      m_DistanceSensor = intake2.getAnalog(Mode.kAbsolute);
    }
    intake2.follow(intake1);
    intake2.setInverted(true);
    intake1.setInverted(false);
    intake1.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intakeSideRight.setInverted(false);
    intakeSideLeft.setInverted(true);
    intakeSideLeft.setIdleMode(IdleMode.kCoast);
    intakeSideRight.setIdleMode(IdleMode.kCoast);
    intake1.setSmartCurrentLimit(25, 40, 1000);
    intake2.setSmartCurrentLimit(25, 40, 1000);
    intakeSideLeft.setSmartCurrentLimit(25, 40, 1000);
    intakeSideRight.setSmartCurrentLimit(25, 40, 1000);
    intake1.burnFlash();
    intake2.burnFlash();
    intakeSideLeft.burnFlash();
    intakeSideRight.burnFlash();
  }
  /**
   * sets the intakes speed
   * <p> uses percentage of full power
   * @param intakeRunSpeed percentage of full power, from -1 to 1
   */
  public void intakeYes(double intakeRunSpeed) {
    intake1.set(intakeRunSpeed);
    intakeSideLeft.set(intakeRunSpeed);
    intakeSideRight.set(intakeRunSpeed);
  }

  public void intakeSideWheels(double sideWheelRunSpeed){
    System.out.println("side wheels " + sideWheelRunSpeed);
    intakeSideLeft.set(sideWheelRunSpeed);
    intakeSideRight.set(sideWheelRunSpeed);
  }
  /**
   * sets the intakes speed
   * <p> uses percentage of full power
   * @param intakeRunSpeed NEGATIVE percentage of full power
   */
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
    intakeSideLeft.set(-intakeRunSpeed);
    intakeSideRight.set(-intakeRunSpeed);
  }
  /**
   * sets the intake's power to 0
   */
  public void intakeOff() {
    intake1.set(0);
    intakeSideLeft.set(0);
    intakeSideRight.set(0);
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
  SmartDashboard.putBoolean("is note in", isNoteSeen());
  SmartDashboard.putBoolean("is note held", isNoteHeld());
  }
}
