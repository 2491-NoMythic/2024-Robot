// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;
import frc.robot.settings.Constants.Vision;

/**Contains the data from the Limelight AprilTag pipeline*/
public class LimelightFiducialData {
    LimelightHelpers.Results llresults;
        public boolean isResultValid;
        Pose2d botPoseRed;
        Pose2d botPoseBlue;

        public LimelightFiducialData(Results llresults, boolean valid){
            this.llresults = llresults;
            this.isResultValid = valid;
        }
        public Pose2d getbotPose(){
            if(DriverStation.getAlliance().equals(Alliance.Red)){
                return llresults.getBotPose2d_wpiRed();
            }else{
                return llresults.getBotPose2d_wpiBlue();
            }
        }
        /**
         * @param robotPose the current odometry pose
         * @return true if the most recent vision pose estimation is inside the field and is close enough to the provided pose.
         */
        public boolean isPoseTrustworthy(Pose2d robotPose){
            Pose2d poseEstimate = this.getbotPose();
            if ((poseEstimate.getX()<Vision.fieldCorner.getX() && poseEstimate.getY()<Vision.fieldCorner.getY()) //Don't trust estimations that are outside the field perimeter.
                && robotPose.getTranslation().getDistance(poseEstimate.getTranslation()) < 0.5) //Dont trust pose estimations that are more than half a meter from current pose.
                return true;
            else return false;
        }
        public double gettimestamp(){
            return (Timer.getFPGATimestamp()
            - (llresults.latency_capture / 1000.0)
            - (llresults.latency_pipeline / 1000.0));
        }
}
