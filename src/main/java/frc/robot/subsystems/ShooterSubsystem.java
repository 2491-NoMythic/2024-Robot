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

import frc.robot.settings.Constants;
import  frc.robot.settings.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax shooter1;
  CANSparkMax shooter2;

  double runSpeed;

  RelativeEncoder encoder1;

    /** Creates a new Shooter. */
  public ShooterSubsystem(double runSpeed) {
    SparkPIDController shooterPID;
    shooter1 = new CANSparkMax(ShooterConstants.SHOOTER_1_MOTORID, MotorType.kBrushless);
    shooter2 = new CANSparkMax(ShooterConstants.SHOOTER_2_MOTORID, MotorType.kBrushless);
    
    shooter1.restoreFactoryDefaults();
    shooter2.follow(shooter1);
    shooter2.setInverted(true);
    

    encoder1 = shooter1.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);

    shooterPID = shooter1.getPIDController();
     double kP = Constants.ShooterConstants.kP;
     double kI = Constants.ShooterConstants.kI;
     double kD = Constants.ShooterConstants.kD;
     double kIz = Constants.ShooterConstants.kIz;
     double kFF = Constants.ShooterConstants.kFF;
     double kMaxOutput = Constants.ShooterConstants.kMaxOutput;
     double kMinOutput = Constants.ShooterConstants.kMinOutput;


    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);  


    //Adjust the value runSpeed later. (SmartDashboard stuff)
    shooterPID.setReference(runSpeed, ControlType.kVelocity);
    




  }
  

  public void shootThing(double runSpeed) {
    shooter1.set(runSpeed);
  }
  public void turnOff(){
    shooter1.set(0);
  }
}
