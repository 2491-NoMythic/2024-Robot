package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Vision;
import frc.robot.settings.Constants;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class DriverAssist extends Command {
 DrivetrainSubsystem drivetrain;
  LimelightDetectorData detectorData;
  Limelight limelight;
  double runsInvalid;

  PIDController txController;
  PIDController tyController;
  SlewRateLimiter tyLimiter;

  double tx;
  double ty;
  /** Creates a new CollectNote. */
  public DriverAssist(DrivetrainSubsystem drivetrain, Limelight limelight) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    runsInvalid = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void initialize() {
    runsInvalid = 0;
    txController = new PIDController(
        // Vision.K_DETECTOR_TX_P,
        0.035,//0.03,
        Vision.K_DETECTOR_TX_I,
        Vision.K_DETECTOR_TX_D);
    tyController = new PIDController(
        0.6,//Vision.K_DETECTOR_TY_P,
        Vision.K_DETECTOR_TY_I,
        Vision.K_DETECTOR_TY_D);
    tyLimiter = new SlewRateLimiter(20, -20, 0);
    txController.setSetpoint(0);
    tyController.setSetpoint(0);
    txController.setTolerance(3.5, 0.25);
    tyController.setTolerance(1, 0.25);
    detectorData = limelight.getNeuralDetectorValues();
    SmartDashboard.putBoolean("note seen", detectorData.isResultValid);
  }

  public void execute(){

    //set the speed to which it adjusts its horizontal speed proportional to the local forward velocity the robot is going
    txController.setP(0.035 * drivetrain.getLocalForwardRobotVelocity()/1.2); //1.2 is an arbitrary number.

    detectorData = limelight.getNeuralDetectorValues();
    if (detectorData == null) {
      drivetrain.stop();
      System.err.println("nullDetectorData");
      return;
    }

    if (!detectorData.isResultValid) { // if the data isnt valid
      if (runsInvalid <= 10) { // don't stop imediately, in case only a couple frames were missed
        drivetrain.drive(new ChassisSpeeds(tyLimiter.calculate(0), 0, 0));
      } else {
        drivetrain.stop();
      }
      System.err.println("invalidDetectorData");
      runsInvalid++;
      return;
    } else {
      runsInvalid = 0;
    }

    ty = detectorData.ty;
 
    tx = detectorData.tx;
    double forwardSpeed = tyLimiter.calculate(-20/Math.abs(tx));
    if(Math.abs(forwardSpeed)>1) {forwardSpeed = -1;}
    drivetrain.drive(new ChassisSpeeds( forwardSpeed, txController.calculate(-tx), 0));
    SmartDashboard.putNumber("CollectNote/forward speed limited", forwardSpeed);

        
    SmartDashboard.putNumber("CollectNote/calculated radians per second", txController.calculate(tx));
    SmartDashboard.putNumber("CollectNote/calculated forward meters per second", tyLimiter.calculate(-20/Math.abs(tx)));
    SmartDashboard.putNumber("CollectNote/runsInvalid", runsInvalid);
  }
}
