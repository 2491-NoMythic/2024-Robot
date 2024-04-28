package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kI;
import static frc.robot.settings.Constants.ShooterConstants.AUTO_AIM_ROBOT_kP;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AimRobotMoving extends Command {
    DrivetrainSubsystem m_drivetrain;
    DoubleSupplier desiredRobotAngleSupplier;
    double desiredRobotAngle;
    double currentHeading;
    double differenceAngle;
    double turningSpeed;
    PIDController speedController;
    DoubleSupplier translationXSupplier;
    DoubleSupplier translationYSupplier;
    BooleanSupplier run;
    BooleanSupplier PodiumAngleSup;
    BooleanSupplier FarStageAngleSup;
    DoubleSupplier rotationSupplier;
    double rotationSpeed;
    double allianceOffset;
    BooleanSupplier SubwooferAngleSup;
    BooleanSupplier OverStagePassSup;
    BooleanSupplier OppositeStageShotSup;
    BooleanSupplier AutonomousLongShotSup;
  /**
   * a command to automatically aim the robot at the speaker if odometry is correct. This command also controls aiming from setpoints incase the odometry isn't working.
   * @param drivetrain the swerve drive subsystem
   * @param rotationSupplier rotational throttle, from -1 to 1
   * @param translationXSupplier the forward throttle, from -1 to 1
   * @param translationYSupplier the sideways (to the right) throttle, from -1 to 1
   * @param run the button used to trigger this command
   * @param PodiumAngleSup the button to turn the robot to poidum setpoint
   * @param FarStageAngleSup the button to turn the robot to the far stage leg setpoint
   * @param SubwooferAngleSup the button to use the subwoofer setpoint (doesn't turn the robot, but stops the robot from auto-aiming when you rev up)
   * @param OverStagePassSup button to press to aim at the speaker from the opposite side of the stage (from the speaker)
   * @param OppositeStageShotSup button to press to aim at the speaker from the opposite side of the stage (from the speaker)
   */
  public AimRobotMoving(DrivetrainSubsystem drivetrain, DoubleSupplier rotationSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, BooleanSupplier run, BooleanSupplier PodiumAngleSup, BooleanSupplier FarStageAngleSup, BooleanSupplier SubwooferAngleSup, BooleanSupplier OverStagePassSup, BooleanSupplier OppositeStageShotSup, BooleanSupplier AutonomousLongShotSup){
        m_drivetrain = drivetrain;
        speedController = new PIDController(
          AUTO_AIM_ROBOT_kP, 
          AUTO_AIM_ROBOT_kI,
          AUTO_AIM_ROBOT_kD);
          speedController.setTolerance(ShooterConstants.ROBOT_ANGLE_TOLERANCE);
          speedController.enableContinuousInput(-180, 180);
          this.translationXSupplier = translationXSupplier;
          this.translationYSupplier = translationYSupplier;
          this.SubwooferAngleSup = SubwooferAngleSup;
          this.rotationSupplier = rotationSupplier;
          this.FarStageAngleSup = FarStageAngleSup;
          this.PodiumAngleSup = PodiumAngleSup;
          this.OverStagePassSup = OverStagePassSup;
          this.OppositeStageShotSup = OppositeStageShotSup;
          this.run = run;
          this.AutonomousLongShotSup = AutonomousLongShotSup;
          addRequirements(drivetrain);
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
          SmartDashboard.putBoolean("isRotateRunning", true);
          
        }
        
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          desiredRobotAngle = m_drivetrain.calculateSpeakerAngleMoving();
          double podiumRobotAngle;
          double farStageRobotAngle;
          double OverStagePassAngle;
          double OppositeStageShotAngle;
          double AutonomousLongShotAngle;
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            podiumRobotAngle = Field.RED_PODIUM_ROBOT_ANGLE;
            farStageRobotAngle = Field.RED_FAR_STAGE_ROBOT_ANGLE;
            OverStagePassAngle = Field.RED_OVER_STAGE_PASS_ANGLE;
            OppositeStageShotAngle = Field.RED_OPPOSITE_STAGE_ROBOT_ANGLE;
            AutonomousLongShotAngle = 35;
          } else {
            podiumRobotAngle = Field.BLUE_PODIUM_ROBOT_ANGLE;
            farStageRobotAngle = Field.BLUE_FAR_STAGE_ROBOT_ANGLE;
            OverStagePassAngle = Field.BLUE_OVER_STAGE_PASS_ANGLE;
            OppositeStageShotAngle = Field.BLUE_OPPOSITE_STAGE_ROBOT_ANGLE;
            AutonomousLongShotAngle = 146.3;
          }
          if(PodiumAngleSup.getAsBoolean()) {
            speedController.setSetpoint(podiumRobotAngle);
          } else if(FarStageAngleSup.getAsBoolean()) {
            speedController.setSetpoint(farStageRobotAngle);
          } else if(OverStagePassSup.getAsBoolean()) {
            speedController.setSetpoint(OverStagePassAngle);
          } else if(OppositeStageShotSup.getAsBoolean()) {
            speedController.setSetpoint(OppositeStageShotAngle);
          }  else if (AutonomousLongShotSup.getAsBoolean()) {
            speedController.setSetpoint(AutonomousLongShotAngle);
          } else {
            speedController.setSetpoint(desiredRobotAngle);
          }
          //move robot to desired angle
          this.currentHeading = m_drivetrain.getOdometryRotation().getDegrees();
          SmartDashboard.putNumber("AIMROBOT/input degrees", currentHeading);
          SmartDashboard.putNumber("AIMROBOT/setpoint", speedController.getSetpoint());
          if(Math.abs(rotationSupplier.getAsDouble()) > 0.3) {
            rotationSpeed = rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
          } else {
            rotationSpeed = speedController.calculate(currentHeading);
          }
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            allianceOffset = Math.PI;
          } else {
            allianceOffset = 0;
          }
          if(!SubwooferAngleSup.getAsBoolean()){
              m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
              translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
              translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
              rotationSpeed,
              new Rotation2d(m_drivetrain.getPose().getRotation().getRadians()+allianceOffset)));
          }
        // m_drivetrain.drive(new ChassisSpeeds(
        //   translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        //   translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        //   speedController.calculate(differenceAngle)));

        SmartDashboard.putNumber("current Heading", m_drivetrain.getPose().getRotation().getDegrees()%360);
        SmartDashboard.putNumber("difference", differenceAngle);
        SmartDashboard.putNumber("desired angle", desiredRobotAngle);
        SmartDashboard.putNumber("PID calculated output", speedController.calculate(differenceAngle));
    
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    SmartDashboard.putBoolean("isRotateRunning", false);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!run.getAsBoolean());
  }
}
