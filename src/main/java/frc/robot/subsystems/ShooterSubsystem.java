 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 
 package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
 import frc.robot.commands.AngleShooter;
import frc.robot.commands.RotateRobot;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.Constants;
import  frc.robot.settings.Constants.ShooterConstants;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

import java.util.function.DoubleSupplier;

 
 public class ShooterSubsystem extends SubsystemBase {
   TalonFX shooterR;
   TalonFX shooterL;
   double runSpeed;
   TalonFXConfigurator configuratorR;
   TalonFXConfigurator configuratorL;

  double differenceAngle;
	 double currentHeading;
	 double m_DesiredShooterAngle;
   double targetVelocityL;
   double targetVelocityR;
   int revStateL;
   int revStateR;
 
  CurrentLimitsConfigs currentLimitConfigs;
   Slot0Configs PIDLeftconfigs; 
   Slot0Configs PIDRightconfigs; 
   RelativeEncoder encoder1;

   //speaker angle calculating variables:

	double turningSpeed;
	RotateRobot rotateRobot;
	AngleShooter angleShooter;
	int accumulativeTurns;
  int runsValid;

  MotorLogger motorLoggerR;
  MotorLogger motorLoggerL;
  
  /** Creates a new Shooter. */
  public ShooterSubsystem(double runSpeed) {
    if(Preferences.getBoolean("CompBot", true)) {
      PIDRightconfigs = new Slot0Configs().withKP(ShooterConstants.CompRightkP).withKV(ShooterConstants.CompRightkFF).withKI(0.004);
      PIDLeftconfigs = new Slot0Configs().withKP(ShooterConstants.CompLeftkP).withKV(ShooterConstants.CompLeftkFF).withKI(0.004);
  } else {
      PIDRightconfigs = new Slot0Configs().withKP(ShooterConstants.PracRightkP).withKV(ShooterConstants.PracRightkFF).withKI(0);
      PIDLeftconfigs = new Slot0Configs().withKP(ShooterConstants.PracLeftkP).withKV(ShooterConstants.PracLeftkFF).withKI(0);
    }
    runsValid = 0;
    shooterR = new TalonFX(ShooterConstants.SHOOTER_R_MOTORID);
    shooterL = new TalonFX(ShooterConstants.SHOOTER_L_MOTORID);
    shooterL.getConfigurator().apply(new TalonFXConfiguration());
    shooterR.getConfigurator().apply(new TalonFXConfiguration());
    shooterL.setInverted(true);
    shooterR.setInverted(false);
    shooterL.setControl(shooterR.getAppliedControl());
    shooterL.setNeutralMode(NeutralModeValue.Coast);
    shooterR.setNeutralMode(NeutralModeValue.Coast);
    
    configuratorR = shooterR.getConfigurator();
    configuratorL = shooterL.getConfigurator();
    configuratorL.apply(new FeedbackConfigs().withSensorToMechanismRatio(1).withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor).withRotorToSensorRatio(1));
    configuratorR.apply(new FeedbackConfigs().withSensorToMechanismRatio(0.5).withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor).withRotorToSensorRatio(1));
    
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    
    configuratorR.apply(PIDRightconfigs);
    configuratorL.apply(PIDLeftconfigs);

    DataLog log = DataLogManager.getLog();
    motorLoggerL = new MotorLogger(log, "/shooter/motorL");
    motorLoggerR = new MotorLogger(log, "/shooter/motorR");
  }
  /**
   * configures the current limits on the shooter motors. Can be ran as many times as you want
   * @param supplyLimit the supply limit to apply to the motors
   * @param statorLimit the stator limit to apply to the motors
   */
  private void adjCurrentLimit( double supplyLimit, double statorLimit){
    currentLimitConfigs.SupplyCurrentLimit = supplyLimit;
    currentLimitConfigs.StatorCurrentLimit = statorLimit;
    configuratorL.apply(currentLimitConfigs);
    configuratorR.apply(currentLimitConfigs);
  }
  public void setTargetVelocity(double RPS_Left, double RPS_Right, double supplyLimit, double statorLimit) {
    targetVelocityL = RPS_Left;
    targetVelocityR = RPS_Right;
    adjCurrentLimit(supplyLimit, statorLimit);
  }
  //public void shootThing(double runSpeed) {
  //  currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
  //  shooterR.set(runSpeed);
  //  shooterL.set(runSpeed);
  //}
  /**
   * configures the motor's current limits to our desired limit's while shooting, and then sets the setpoint of both shooter motor's onboard PID loop. The left motor will run at half the desired speed, and the right motor will run at full. To run
   * the motors at the same speed, use {@code shooter.shootSameRPS(RPS)}
   * @param RPS the desired Rotations Per Second
   */
    public void shootRPS(double RPS) {
      shootRPSWithCurrent(RPS, ShooterConstants.CURRENT_LIMIT, 100);
    }
    public void shootSameRPS(double RPS) {
      setTargetVelocity(RPS, RPS, ShooterConstants.CURRENT_LIMIT, 100);
    }
    public void shootSameRPSWithCurrent(double RPS, double supplyLimit, double statorLimit) {
      setTargetVelocity(RPS, RPS, supplyLimit, statorLimit);
    }
   /**
     * allows you to set the shooter's speed using a supplier. this way you can use a value on SmartDashboard to tune
     * the shooter's speed. 
     * <p> if sameSpeed is true, than the wheels will run at the same RPS. Otherwise, the left wheel will run at half the speed.
     * @param RPSSup
     * @param sameSpeed
     */
    public void shootWithSupplier(DoubleSupplier RPSSup, boolean sameSpeed) {
      DoubleSupplier speedSupplier = RPSSup;
      if(sameSpeed) {
        shootSameRPS(speedSupplier.getAsDouble());
      } else {
        shootRPS(speedSupplier.getAsDouble());
      }
    }
   /**
     * configures the motor's current limits, then it set's the setpoint for the motor's onboard PID loop based on the RPS
     * the left motor will run at half the RPS, and the right motor will run at te full RPS
     * @param RPS the desired Rotations Per Second of the shooter wheels
     * @param supplyLimit the desired SupplyCurrentLimit for the shooter motor's
     * @param statorLimit the desired StatorCurrentLimit for the shooter motor's
     */
    public void shootRPSWithCurrent(double RPS, double supplyLimit, double statorLimit){
      setTargetVelocity(RPS/2, RPS, supplyLimit, statorLimit);
    }
    /**
     * returns the absolute value of the right shooter motor's onboard PID loop. AKA it tells you how far away the right shooter motor's speed is from what we've told it to be
     * <p> We use the right shooter motor becuase it's geared, so it takes longer to rev up or down
     * @return the speed error of the right shooter motor
     */
    private double getError() {
      return Math.abs(getSignedError());
    }

    private double getSignedError(){
      return shooterR.getVelocity().getValueAsDouble()-targetVelocityR;
    }
    /**
     * checks if the right shooter motor is attempting to rev up
    * <p> We use the right shooter motor becuase it's geared, so it takes longer to rev up or down.
     * @return Is the speed of the right wheels higher than 15 RPS
     */
    public boolean isReving() {
      return shooterR.getVelocity().getValueAsDouble()>15;
    }
    /**
     * checks if the shooter has been within ALLOWED_SPEED_ERROR (from constants) for more than 20 (from constants) commandScheduler loops
     * @return has the shooter been at speed for long enough
     */
    public boolean validShot() {
      return runsValid >= Constants.LOOPS_VALID_FOR_SHOT;
    }
    /**
     * turns off both of the shooter motors. We use percentage-of-full-power so that the wheels will coast to a stop and not use unnecessary power to stop them instantly.
     */
  public void turnOff(){
    targetVelocityL = 0;
    targetVelocityR = 0;
    updateMotors();
  }

  /**
   * Sets the motor to Rev if below target speed.
   * @param isReving Old isReving
   * @return Updated isReving
   */
  private static int updateIsReving(int revState, double targetSpeed, double speed){
    double revUpEn =  10;
    double revUpDis = 2;
    double revDownEn = -10;
    double revDownDis = 2;
    double difference = targetSpeed - speed;
    SmartDashboard.putNumber("SHOOTER/desired speed difference", difference);
    if (difference>revUpEn){
      revState = 1;
    } else if (difference<revDownEn && targetSpeed<10){
      revState = -1;
    } else if(revState != 0 && Math.abs(difference)<revDownDis) {
      revState = 0;
    }
    return revState;
  }
  /**
   * If the motor is Reving, set speed to full, if not Reving, use PID Mode
   */
  private static void updateMotor(TalonFX shooterMotor, int revState, double targetSpeed, CurrentLimitsConfigs currentLimits){
    currentLimits.StatorCurrentLimitEnable = revState != -1;
    shooterMotor.getConfigurator().apply(currentLimits);
    if(targetSpeed == 0) {
      shooterMotor.set(0);
    } else if(targetSpeed<0) {
      shooterMotor.setControl(new VelocityDutyCycle(targetSpeed).withSlot(0));
    } else if (revState != 0) {
      shooterMotor.set(revState);
    } else {
      shooterMotor.setControl(new VelocityDutyCycle(targetSpeed).withSlot(0));
    }
  }
  /**
   *  Revs left and right motors while below target velocity, if above target velocity, the motors are put to PID Mode
   */
  private void updateMotors(){
    revStateL = updateIsReving(revStateL, targetVelocityL, shooterL.getVelocity().getValueAsDouble());
    revStateR = updateIsReving(revStateR, targetVelocityR, shooterR.getVelocity().getValueAsDouble());
    updateMotor(shooterL, revStateL, targetVelocityL, currentLimitConfigs); 
    updateMotor(shooterR, revStateR, targetVelocityR, currentLimitConfigs); 
  }
  public double getRSpeed() {
    return shooterR.getVelocity().getValueAsDouble();
  }
  public double getLSpeed() {
    return shooterL.getVelocity().getValueAsDouble();
  }
  // public double getSpeedRPS() {
  //   return shooterR.getVelocity().asSupplier().get();
  // }
@Override
  public void periodic() {
    updateMotors();
    
    SmartDashboard.putNumber("TESTING shooter speed error", getError());
    SmartDashboard.getBoolean("shooter speed rev'ed", validShot());
    SmartDashboard.putNumber("shooter current right", shooterR.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("shooter current left", shooterL.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Right shooter speed", shooterR.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Left shooter speed", shooterL.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Left shooter revState", revStateL);
    SmartDashboard.putNumber("Shooter/right shooter revState", revStateR);
    SmartDashboard.putNumber("Shooter/right integral value", shooterR.getClosedLoopIntegratedOutput().getValueAsDouble());
    double error = getSignedError();
    RobotState.getInstance().ShooterError = error;
    if(Math.abs(error)<ShooterConstants.ALLOWED_SPEED_ERROR) {
      runsValid++;
    } else {
      runsValid = 0;
    }
    motorLoggerL.log(shooterL);
    motorLoggerR.log(shooterR);
  }
}
 
