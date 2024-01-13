// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import  frc.robot.settings.Constants.ShooterConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax shooter1;
  CANSparkMax shooter2;
  CANSparkMax pitchMotor;
  double runSpeed;
  

  /** Creates a new Shooter. */
  public ShooterSubsystem(double runSpeed) {
    shooter1 = new CANSparkMax(ShooterConstants.SHOOTER_1_MOTORID, MotorType.kBrushless);
    shooter2 = new CANSparkMax(ShooterConstants.SHOOTER_2_MOTORID, MotorType.kBrushless);
  }
  

  public void shootThing(double runSpeed) {
    shooter1.set(runSpeed);
    shooter2.set(-runSpeed);
  }

  public void turnOff(){
    shooter1.set(0);
    shooter2.set(0);
  }

  public void pitchShooter(double pitchSpeed){
    pitchMotor.set(pitchSpeed);
  }
}
