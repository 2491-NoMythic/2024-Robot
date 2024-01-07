package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.settings.LimelightFiducialData;
import frc.robot.settings.LimelightValues;
import frc.robot.settings.Constants.Vision;
public class Limelight {

    private static Limelight limelight;

    public static Boolean aprilTagEnabled = false;
    public static Boolean aprilTagForceTrust = false;
    public static Boolean detectorEnabled = false;
    
    public static LimelightFiducialData latestAprilTagValues;
    public static LimelightDetectorData latestDetectorValues;

    private  Limelight(){
      vision_thread();
    }

    public static Limelight getInstance(){
        if (limelight == null){
            limelight = new Limelight();
        }
        return limelight;
    }
    public static void useAprilTagLimelight(boolean enabled) {
      aprilTagEnabled = enabled;
    }
    public static void forceTrustAprilTag(boolean enabled) {
      aprilTagEnabled = enabled;
    }
    public static void useDetectorLimelight(boolean enabled) {
      aprilTagEnabled = enabled;
    }
    private LimelightFiducialData getAprilTagValues(){
        return new LimelightFiducialData(LimelightHelpers.getLatestResults(Vision.APRILTAG_LIMELIGHT_NAME).targetingResults, LimelightHelpers.getTV(Vision.APRILTAG_LIMELIGHT_NAME));
    }
    private LimelightDetectorData getNeuralDetectorValues(){
        return new LimelightDetectorData(LimelightHelpers.getLatestResults(Vision.OBJ_DETECITON_LIMELIGHT_NAME).targetingResults, LimelightHelpers.getTV(Vision.OBJ_DETECITON_LIMELIGHT_NAME));
    }
    
    public void vision_thread(){
        try{
          new Thread(() -> { // this will make a seperate thread for limelight things to happen on and not interrup other code 
            while(true){
              periodic();
              try {
                Thread.sleep(20);
              } catch (InterruptedException e) {
                e.printStackTrace();
              }
            }
          }).start();
        }catch(Exception e){}
      }
    
      public void periodic() {
        if (aprilTagEnabled) Limelight.latestAprilTagValues = getAprilTagValues();
        if (detectorEnabled) Limelight.latestDetectorValues = getNeuralDetectorValues();
      }
}