// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightValues {
        LimelightHelpers.Results llresults;
        public boolean isResultValid;
        int numTags;
        double[] tx = new double[5];
        double[] ty = new double[5];
        double[] ta = new double[5];
        Pose2d botPoseRed;
        Pose2d botPoseBlue;

        private final Translation2d fieldCorner = new Translation2d(16.54, 8.02);

        public LimelightValues(Results llresults, boolean valid){
            this.llresults = llresults;
            this.isResultValid = valid;
            if (isResultValid) {
                this.numTags = llresults.targets_Fiducials.length;
                for (int i = 0; i < numTags; i++) {
                    this.tx[i] = llresults.targets_Fiducials[i].tx;
                    this.ty[i] = llresults.targets_Fiducials[i].ty;
                    this.ta[i] = llresults.targets_Fiducials[i].ta;
                }
                this.botPoseRed = llresults.getBotPose2d_wpiRed();
                this.botPoseBlue = llresults.getBotPose2d_wpiBlue();
            }
        }
        public double gettx(int index){return tx[index];}
        public double getty(int index){return ty[index];}
        public double getta(int index){return ta[index];}
        public Pose2d getbotPose(){
            if(DriverStation.getAlliance().equals(Alliance.Red)){//TODO make sure this works
            // if(DriverStation.getAlliance() ==  Alliance.Red){
                return botPoseRed;
            }else{
                return botPoseBlue;
            }
        }
        public boolean isPoseTrustworthy(Pose2d robotPose){
            Pose2d poseEstimate = this.getbotPose();
            if ((poseEstimate.getX()<fieldCorner.getX() && poseEstimate.getY()<fieldCorner.getY()) //Don't trust estimations that are outside the field perimeter.
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
