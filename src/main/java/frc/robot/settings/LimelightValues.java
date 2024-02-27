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

import frc.robot.settings.Constants.Vision;
import static frc.robot.settings.Constants.Vision.MAX_TAG_DISTANCE;
import static frc.robot.settings.Constants.Vision.APRILTAG_CLOSENESS;
import static frc.robot.settings.Constants.Vision.limelightLensHeightInches;
import static frc.robot.settings.Constants.Vision.limelightMountAngleDegrees;
import static frc.robot.settings.Constants.Vision.AprilTagHeight;

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
        double tagDistance;

        private final Translation2d fieldCorner = new Translation2d(16.54, 8.02);

        public LimelightValues(Results llresults, boolean valid){
            this.llresults = llresults;
            this.isResultValid = valid;
            this.tagDistance = 30;
            if (isResultValid) {
                this.numTags = llresults.targets_Fiducials.length;
                for (int i = 0; i < numTags; i++) {
                    this.tx[i] = llresults.targets_Fiducials[i].tx;
                    this.ty[i] = llresults.targets_Fiducials[i].ty;
                    this.ta[i] = llresults.targets_Fiducials[i].ta;
                    // this.tagDistance = llresults.targets_Fiducials[0].getTargetPose_RobotSpace2D().getTranslation().getNorm();
                }
                this.botPoseRed = llresults.getBotPose2d_wpiRed();
                this.botPoseBlue = llresults.getBotPose2d_wpiBlue();
                // SmartDashboard.putNumber("VISION dist to apriltag [0]", tagDistance);
                // SmartDashboard.putNumber("VISION dist to apriltag [1]", llresults.targets_Fiducials[1].getTargetPose_RobotSpace2D().getTranslation().getNorm());
                // SmartDashboard.putNumber("VISION dist to apriltag [2]", llresults.targets_Fiducials[2].getTargetPose_RobotSpace2D().getTranslation().getNorm());
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
        public Pose2d getBotPoseBlue() {
            return botPoseBlue;
        }
        public double calculateTagDistance(String limelightName) {
            if(isResultValid) {
                double targetOffsetAngle_Vertical = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty").getDouble(0.0);

                double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
                //calculate distance
                double distanceFromLimelightToGoalInches = (Vision.AprilTagHeight - Vision.limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                return distanceFromLimelightToGoalInches;
            } else {
                return 30;
            }
        }
        // public double getTagDistance() {
        //     return tagDistance;
        // }
        public boolean isPoseTrustworthy(double tagDist){
            Pose2d poseEstimate = this.botPoseBlue;
            if ((poseEstimate.getX()<fieldCorner.getX() && poseEstimate.getY()<fieldCorner.getY()) //Don't trust estimations that are outside the field perimeter.
                && tagDist<=MAX_TAG_DISTANCE) { //Dont trust pose estimations if the april tag is more than X meters away
                return true;
            } else {
                return false;
            }
        }
        public double gettimestamp(){
            return (Timer.getFPGATimestamp()
            - (llresults.latency_capture / 1000.0)
            - (llresults.latency_pipeline / 1000.0));
        }
}
