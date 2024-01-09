// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import  frc.robot.settings.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax Shooter1;
  CANSparkMax Shooter2;
  double runSpeed;
  /** Creates a new Shooter. */
  public ShooterSubsystem(double runSpeed) {
    CANSparkMax Shooter1 = new CANSparkMax(ShooterConstants.SHOOTER_1_MOTORID, MotorType.kBrushless);
    CANSparkMax Shooter2 = new CANSparkMax(ShooterConstants.SHOOTER_2_MOTORID, MotorType.kBrushless);
  }
  

  public void shootThing(double runSpeed) {
    Shooter1.set(runSpeed);
    Shooter2.set(runSpeed);
  }
  public void turnOff(){
    Shooter1.set(0);
    Shooter2.set(0);
  }
}
