// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
 import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  CANSparkMax climbMotor;
  RelativeEncoder climbEncoder;
  double speed;
  /** Creates a new Climber. */
  public Climber(double speed) {
    climbMotor = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR, MotorType.kBrushless);
    climbEncoder = climbMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4098);
  }
 public void climberGo(double speed){
  climbMotor.set(speed);
 }

}
