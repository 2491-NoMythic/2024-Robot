// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  CANSparkMax climbMotorR;
  CANSparkMax climbMotorL;
  RelativeEncoder climbEncoderR;
  RelativeEncoder climbEncoderL;
  double speed;
  double voltage;
  double currentEncoderRotationsL;
  double currentEncoderRotationsR;
  SparkLimitSwitch limitSwitchR;
  SparkLimitSwitch limitSwitchL;
  double initialEncoderRotationsL;
  double initialEncoderRotationsR;
  /** Creates a new Climber. */
  public Climber() {
    climbMotorR = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
    climbMotorL = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    climbEncoderR = climbMotorR.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4098);
    climbEncoderL = climbMotorL.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4098);
    limitSwitchR = climbMotorR.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    limitSwitchL = climbMotorL.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    initialEncoderRotationsL = Math.abs(climbEncoderL.getPosition());
    initialEncoderRotationsL = Math.abs(climbEncoderR.getPosition());
    climbMotorL.setIdleMode(IdleMode.kBrake);
    climbMotorR.setIdleMode(IdleMode.kBrake);
  }
 public void climberGo(double speed){
  if (speed<0) {
    climbMotorL.set(speed);
    climbMotorR.set(speed);
  } else {
    if (currentEncoderRotationsL < ClimberConstants.MAX_MOTOR_ROTATIONS){
    climbMotorL.set(speed);}
    if (currentEncoderRotationsR < ClimberConstants.MAX_MOTOR_ROTATIONS){
    climbMotorR.set(speed);}
  }
 }

 public void climberStop(){
  climbMotorR.set(0);
  climbMotorL.set(0);
 }

 public void climberVoltage(double voltage){
  climbMotorR.setVoltage(voltage);
  climbMotorL.setVoltage(voltage);
 }

 public double getRightRPM(){
  return climbEncoderR.getVelocity();
 }

 public double getLeftRPM(){
  return climbEncoderL.getVelocity();
 }

 public boolean isClimberIn(){
  return limitSwitchR.isPressed() && limitSwitchL.isPressed();
}
@Override
public void periodic() {
  currentEncoderRotationsL = Math.abs(climbEncoderL.getPosition()) - initialEncoderRotationsL;
  currentEncoderRotationsR = Math.abs(climbEncoderR.getPosition()) - initialEncoderRotationsR;
}
}
