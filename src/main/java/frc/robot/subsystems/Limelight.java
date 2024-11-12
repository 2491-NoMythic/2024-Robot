package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.helpers.MythicalMath;
import frc.robot.settings.LimelightDetectorData;

import static frc.robot.settings.Constants.Vision.*;

import java.io.IOException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Limelight {

    private static Limelight limelight;

    private static Field2d field1 = new Field2d();
    private static Field2d field2 = new Field2d();

    public static Boolean detectorEnabled = false;

     public static final AprilTagFieldLayout FIELD_LAYOUT;

      static {
            try {
                FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

    private Limelight() {
        SmartDashboard.putBoolean("Vision/Left/valid", false);
        SmartDashboard.putBoolean("Vision/Left/trusted", false);
        SmartDashboard.putBoolean("Vision/Right/valid", false);
        SmartDashboard.putBoolean("Vision/Right/trusted", false);
        SmartDashboard.putData("Vision/Left/pose", field1);
        SmartDashboard.putData("Vision/Right/pose", field2);


        //logs active selected path 
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
              Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
    }

    public static Limelight getInstance() {
        if (limelight == null) {
            limelight = new Limelight();
        }
        return limelight;
    }

    public static void useDetectorLimelight(boolean enabled) {
        detectorEnabled = enabled;
    }

    /**
     * Gets the most recent limelight pose estimate, given that a trustworthy
     * estimate is
     * available. Uses the provided odometryPose for additional filtering.
     * <p>
     * Trusted poses must:
     * <ul>
     * <li>Be within field bounds.
     * <li>Have an average tag distance within [MAX_TAG_DISTANCE] from the robot.
     * <li>Be within [ALLOWABLE_POSE_DIFFERENCE] from the given odometryPose.
     * </ul>
     * 
     * @param odometryPose The current odometry pose estimate
     * @return A valid and trustworthy pose. Null if no valid pose. Poses are
     *         prioritized by lowest tagDistance.
     */
    public PoseEstimate getTrustedPose(Pose2d odometryPose) {
        PoseEstimate pose1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT2_NAME);
        PoseEstimate pose2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT3_NAME);

        Boolean pose1Trust = isTrustworthy(APRILTAG_LIMELIGHT2_NAME, pose1, odometryPose);
        Boolean pose2Trust = isTrustworthy(APRILTAG_LIMELIGHT3_NAME, pose2, odometryPose);
        //if the limelight positions will be merged, let SmartDashboard know!
        boolean mergingPoses = false;
        if(pose1Trust&&pose2Trust)  { mergingPoses = true;}
        SmartDashboard.putBoolean("LL poses merged", mergingPoses);

        if (pose1Trust && pose2Trust) {
            return (mergedPose(pose1, pose2)); //merge the two positions proportionally based on the closest tag distance
        } else if (pose1Trust) {
            return pose1;
        } else if (pose2Trust) {
            return pose2;
        } else
            return null;
    }

    public PoseEstimate mergedPose(PoseEstimate pose1, PoseEstimate pose2) {

    // find the coordinates (relative to the robot) of the closest april tag to limelight 2, then limelight 3
        double limelight1y = LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT2_NAME).getY();
		double limelight1x = LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT2_NAME).getX();
		double limelight1z = LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT2_NAME).getZ();

		double limelight2y = LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT3_NAME).getY();
		double limelight2x = LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT3_NAME).getX();
		double limelight2z = LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT3_NAME).getZ();

        // use those coordinates to find the distance to the closest apriltag for each limelight
        double distance1 = MythicalMath.DistanceFromOrigin3d(limelight1x, limelight1y, limelight1z);
        double distance2 = MythicalMath.DistanceFromOrigin3d(limelight2x, limelight2y, limelight2z);
        //ccombines the two positions in an epic, proportional way
        double confidenceSource1 = Math.pow(1/distance1,2);
        double confidenceSource2 = Math.pow(1/distance2,2);
        Pose2d scaledPose1 = MythicalMath.multiplyOnlyPos(pose1.pose, confidenceSource1);
        Pose2d scaledPose2 = MythicalMath.multiplyOnlyPos(pose2.pose, confidenceSource2);
    Pose2d newPose = MythicalMath.divideOnlyPos((MythicalMath.addOnlyPosTogether(scaledPose1, scaledPose2)), (confidenceSource1+confidenceSource2));
    //Pose2d newPose = scaledPose1.plus(new Transform2d(scaledPose2.getTranslation(), new Rotation2d())).div(confidenceSource1+confidenceSource2);
      
    pose1.pose = newPose;
    return pose1;
    }

    public void updateLoggingWithPoses() {
        Pose2d pose1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT2_NAME).pose;
        Pose2d pose2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT3_NAME).pose;

        field1.setRobotPose(pose1);
        field2.setRobotPose(pose2);

        Logger.recordOutput("LeftPose", pose1);
        Logger.recordOutput("RightPose", pose2);

         Pose3d[] AprilTagPoses;

        for (int i = 0; i < 6; i++)
        {
            if(FIELD_LAYOUT.getTagPose(i) == null)
            {
                return;
            }

          //  AprilTagPoses[i] = FIELD_LAYOUT.getTagPose((int) LimelightHelpers.getLimelightNTTableEntry("botpose", APRILTAG_LIMELIGHT2_NAME).getInteger(i));
        }

      //  Logger.recordOutput("AprilTagVision", );
        Logger.recordOutput("AprilTagVision", LimelightHelpers.getLimelightNTTableEntry("botpose", APRILTAG_LIMELIGHT2_NAME).getDoubleArray(new double[6]));
        Logger.recordOutput("Vision/targetposes/LeftPose/CameraSpace", LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT2_NAME));
        Logger.recordOutput("Vision/targetposes/RightPose/CameraSpace", LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHT3_NAME));
        Logger.recordOutput("Vision/targetposes/NotePoses/CameraSpace", LimelightHelpers.getTargetPose3d_CameraSpace(OBJ_DETECTION_LIMELIGHT_NAME));
    
        Logger.recordOutput("Vision/targetposes/LeftPose/RobotSpace", LimelightHelpers.getTargetPose3d_RobotSpace(APRILTAG_LIMELIGHT2_NAME));
        Logger.recordOutput("Vision/targetposes/LeftPose/RobotSpace", LimelightHelpers.getTargetPose3d_RobotSpace(APRILTAG_LIMELIGHT3_NAME));
        Logger.recordOutput("Vision/targetposes/NotePoses/RobotSpace", LimelightHelpers.getTargetPose3d_RobotSpace(OBJ_DETECTION_LIMELIGHT_NAME));
    
    }   

    /**
     * Gets the most recent limelight pose estimate, given that a valid estimate is
     * available.
     * <p>
     * Valid estemates are only required to be within the field, so results will be
     * unpredictable.
     * 
     * @return A valid and NOT-ALWAYS-TRUSTWORTHY pose. Null if no valid pose. Poses
     *         are prioritized by lowest tagDistance.
     */
    public PoseEstimate getValidPose() {
        PoseEstimate pose1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT2_NAME);
        PoseEstimate pose2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT3_NAME);

        Boolean pose1Valid = isValid(APRILTAG_LIMELIGHT2_NAME, pose1);
        Boolean pose2Valid = isValid(APRILTAG_LIMELIGHT3_NAME, pose2);

        if (pose1Valid && pose2Valid) {
            return ((pose1.avgTagDist < pose2.avgTagDist) ? pose1 : pose2); //select the limelight that has closer tags.
        } else if (pose1Valid) {
            return pose1;
        } else if (pose2Valid) {
            return pose2;
        } else
            return null;
    }

    public LimelightDetectorData getNeuralDetectorValues() {
        return new LimelightDetectorData(
                LimelightHelpers.getTX(OBJ_DETECTION_LIMELIGHT_NAME),
                LimelightHelpers.getTY(OBJ_DETECTION_LIMELIGHT_NAME),
                LimelightHelpers.getTA(OBJ_DETECTION_LIMELIGHT_NAME),
                LimelightHelpers.getNeuralClassID(OBJ_DETECTION_LIMELIGHT_NAME),
                LimelightHelpers.getTV(OBJ_DETECTION_LIMELIGHT_NAME));
    }

      public LimelightDetectorData getAprilValues(String cameraname) {
        return new LimelightDetectorData(
                LimelightHelpers.getTX(cameraname),
                LimelightHelpers.getTY(cameraname),
                LimelightHelpers.getTA(cameraname),
                LimelightHelpers.getNeuralClassID(cameraname),
                LimelightHelpers.getTV(cameraname));
    }

    public int getLLTagCount(String cameraname)
    {
        return LimelightHelpers.getLatestResults(cameraname).targetingResults.targets_Fiducials.length;
    }



    private boolean isValid(String limelightName, PoseEstimate estimate) {
        Boolean valid = (
                estimate.pose.getX() < FIELD_CORNER.getX() &&
                estimate.pose.getX() > 0.0 &&
                estimate.pose.getY() < FIELD_CORNER.getY() &&
                estimate.pose.getY() > 0.0);

        if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHT2_NAME)) {
            SmartDashboard.putBoolean("Vision/Left/valid", valid);
            SmartDashboard.putNumber("Vision/Left/Stats/valid", (valid ? 1 : 0));
            SmartDashboard.putNumber("Vision/Left/Stats/avgTagDist", estimate.avgTagDist);
            SmartDashboard.putNumber("Vision/Left/Stats/tagCount", estimate.tagCount);
            SmartDashboard.putNumber("Vision/Left/Stats/latency", estimate.latency);
        } else if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHT3_NAME)) {
            SmartDashboard.putBoolean("Vision/Right/valid", valid);
            SmartDashboard.putNumber("Vision/Right/Stats/valid", (valid ? 1 : 0));
            SmartDashboard.putNumber("Vision/Right/Stats/avgTagDist", estimate.avgTagDist);
            SmartDashboard.putNumber("Vision/Right/Stats/tagCount", estimate.tagCount);
            SmartDashboard.putNumber("Vision/Right/Stats/latency", estimate.latency);
        } else {
            System.err.println("Limelight name is invalid. (limelight.isValid)");
        }
        return valid;
    }
    /**
     * checks if the robotPose returned by the limelight is within the field and stable. It does this by running isValid() with the limelight, and checking if 
     * the limelight's pose either contains 2+ tags or is closer then MAX_TAG_DISTANCE (from constants) from the tag.
     * @param limelightName the name of the requested limelight, as seen on NetworkTables
     * @param estimate the poseEstimate from that limelight
     * @param odometryPose the robot's pose from the DriveTrain, unused right now
     * @return true if the pose is within the field bounds and the tag distance is less than 7
     */
    private boolean isTrustworthy(String limelightName, PoseEstimate estimate, Pose2d odometryPose) {
        Boolean trusted = (
                isValid(limelightName, estimate) &&
                // estimate.pose.getTranslation().getDistance(odometryPose.getTranslation()) < ALLOWABLE_POSE_DIFFERENCE && // Unused
                // ((estimate.avgTagDist < MAX_TAG_DISTANCE) || (estimate.tagCount >= 2 && estimate.avgTagDist<6))); // Trust poses when there are two tags, or when the tags are close to the camera.
                estimate.avgTagDist<7);

        if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHT2_NAME)) {
            SmartDashboard.putBoolean("Vision/Left/trusted", trusted);
            SmartDashboard.putNumber("Vision/Left/Stats/trusted", (trusted ? 1 : 0));
        } else if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHT3_NAME)) {
            SmartDashboard.putBoolean("Vision/Right/trusted", trusted);
            SmartDashboard.putNumber("Vision/Right/Stats/trusted", (trusted ? 1 : 0));
        } else {
            System.err.println("Limelight name is invalid. (limelight.isTrustworthy)");
        }
        return trusted;
    }

    public Pose2d getRawLLPose(String LLName)
    {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHT2_NAME).pose;
    }
}