// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 import com.revrobotics.CANSparkMax;
 import com.revrobotics.RelativeEncoder;
 import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.settings.Constants.IntakeConstants;;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake1;
  CANSparkMax intake2;
  CANSparkMax brush1;
  CANSparkMax brush2;
  CANSparkMax brush3;
  CANSparkMax brush;

  double intakeRunSpeed;
  public IntakeSubsystem() {
    CANSparkMax intake1 = new CANSparkMax(IntakeConstants.INTAKE_1_MOTOR, MotorType.kBrushless);
    CANSparkMax intake2 = new CANSparkMax(IntakeConstants.INTAKE_1_MOTOR, MotorType.kBrushless);
    CANSparkMax brush1 = new CANSparkMax(IntakeConstants.BRUSH_1_MOTOR, MotorType.kBrushless);
    CANSparkMax brush2 = new CANSparkMax(IntakeConstants.BRUSH_2_MOTOR, MotorType.kBrushless);
    CANSparkMax brush3 = new CANSparkMax(IntakeConstants.BRUSH_3_MOTOR, MotorType.kBrushless);

    intake2.follow(intake1);
    intake2.setInverted(true);
    intake1.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);
  }

  public void intakeYes(double intakeRunSpeed) {
    intake1.set(intakeRunSpeed);
    brush1.set(intakeRunSpeed);
    brush2.set(intakeRunSpeed);
    brush3.set(intakeRunSpeed);
   
  }
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
    brush1.set(-intakeRunSpeed);
    brush2.set(-intakeRunSpeed);
    brush3.set(-intakeRunSpeed);
  
  }
  public void intakeOff() {
    intake1.set(0);
    brush2.set(0);
    brush1.set(0);
    brush3.set(0);
  }

}
