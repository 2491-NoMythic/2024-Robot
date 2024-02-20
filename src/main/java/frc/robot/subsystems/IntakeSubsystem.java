// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 import com.revrobotics.CANSparkMax;
 import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
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
  SparkAnalogSensor m_DistanceSensor;
  CANSparkMax brush1;
  CANSparkMax brush2;
  CANSparkMax brush3;

  double intakeRunSpeed;
  public IntakeSubsystem() {
    intake1 = new CANSparkMax(IntakeConstants.INTAKE_1_MOTOR, MotorType.kBrushless);
    intake2 = new CANSparkMax(IntakeConstants.INTAKE_2_MOTOR, MotorType.kBrushless);
    intake1.restoreFactoryDefaults();
    intake2.restoreFactoryDefaults();
    m_DistanceSensor = intake2.getAnalog(Mode.kAbsolute);
    if(Preferences.getBoolean("Brushes", false)) {
      brush1 = new CANSparkMax(IntakeConstants.BRUSH_1_MOTOR, MotorType.kBrushless);
      brush2 = new CANSparkMax(IntakeConstants.BRUSH_2_MOTOR, MotorType.kBrushless);
      brush3 = new CANSparkMax(IntakeConstants.BRUSH_3_MOTOR, MotorType.kBrushless);
      brush1.setSmartCurrentLimit(20, 20, 60);
      brush2.setSmartCurrentLimit(20, 20, 60);
      brush3.setSmartCurrentLimit(20, 20, 60);
      brush1.burnFlash();
      brush2.burnFlash();
      brush3.burnFlash();
      brush1.setIdleMode(IdleMode.kCoast);
      brush2.setIdleMode(IdleMode.kCoast);
      brush3.setIdleMode(IdleMode.kCoast);
    }
    intake2.follow(intake1);
    intake2.setInverted(true);
    intake1.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake1.setSmartCurrentLimit(25, 40, 1000);
    intake2.setSmartCurrentLimit(25, 40, 1000);
    intake1.burnFlash();
    intake2.burnFlash();
  }

  public void intakeYes(double intakeRunSpeed) {
    intake1.set(intakeRunSpeed);
    if(Preferences.getBoolean("Brushes", false)) {
      brush1.set(intakeRunSpeed);
      brush2.set(intakeRunSpeed);
      brush3.set(intakeRunSpeed);
    }
   
  }
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
    if(Preferences.getBoolean("Brushes", false)) {
      brush1.set(-intakeRunSpeed);
      brush2.set(-intakeRunSpeed);
      brush3.set(-intakeRunSpeed);
    }
  }
  public void intakeOff() {
    intake1.set(0);
    if(Preferences.getBoolean("Brushes", false)) {
      brush2.set(0);
      brush1.set(0);
      brush3.set(0);
    }
  }
  public boolean isNoteIn() {
    return m_DistanceSensor.getVoltage()<2;
  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("voltage sensor output", m_DistanceSensor.getVoltage());
  SmartDashboard.putBoolean("is note in", isNoteIn());
  }
}
