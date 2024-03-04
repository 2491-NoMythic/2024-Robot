package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.settings.LimelightDetectorData;

import static frc.robot.settings.Constants.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;

public class Limelight {

    private static Limelight limelight;

    public static Boolean detectorEnabled = false;

    public static LimelightDetectorData latestDetectorValues;

    private Limelight() {
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
     * available.
     * <p>
     * Trusted poses must:
     * <ul>
     * <li>Be within field bounds.
     * <li>Have an average tag distance within [MAX_TAG_DISTANCE] from the robot.
     * </ul>
     * 
     * @param odometryPose The current odometry pose estimate
     * @return A valid and trustworthy pose. Null if no valid pose. Poses are
     *         prioritized by lowest tagDistance.
     */
    public PoseEstimate getTrustedPose() {
        PoseEstimate pose1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(APRILTAG_LIMELIGHT2_NAME);
        PoseEstimate pose2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(APRILTAG_LIMELIGHT3_NAME);
        Boolean pose1Trust = false, pose2Trust = false;
        if (pose1 != null) {
            pose1Trust = (pose1.pose.getX() < FIELD_CORNER.getX() &&
                    pose1.pose.getX() > 0.0 &&
                    pose1.pose.getY() < FIELD_CORNER.getY() &&
                    pose1.pose.getY() > 0.0 &&
                    // pose1.tagCount >= 2 &&
                    pose1.avgTagDist < MAX_TAG_DISTANCE);
        }
        if (pose2 != null) {
            pose2Trust = (pose2.pose.getX() < FIELD_CORNER.getX() &&
                    pose2.pose.getX() > 0.0 &&
                    pose2.pose.getY() < FIELD_CORNER.getY() &&
                    pose2.pose.getY() > 0.0 &&
                    // pose2.tagCount >= 2 &&
                    pose2.avgTagDist < MAX_TAG_DISTANCE);
        }
        if (pose1Trust && pose2Trust) {
            return ((pose1.avgTagDist < pose2.avgTagDist) ? pose1 : pose2);
        } else if (pose1Trust) {
            return pose1;
        } else if (pose2Trust) {
            return pose2;
        } else
            return null;
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
        PoseEstimate pose1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(APRILTAG_LIMELIGHT2_NAME);
        PoseEstimate pose2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(APRILTAG_LIMELIGHT3_NAME);

        Boolean pose1Trust = (pose1.pose.getX() < FIELD_CORNER.getX() &&
                pose1.pose.getX() > 0.0 &&
                pose1.pose.getY() < FIELD_CORNER.getY() &&
                pose1.pose.getY() > 0.0 &&
                // pose1.tagCount >= 2 &&
                pose1.pose.getTranslation().getDistance(odometryPose.getTranslation()) < ALLOWABLE_POSE_DIFFERENCE &&
                pose1.avgTagDist < MAX_TAG_DISTANCE);

        Boolean pose2Trust = (pose2.pose.getX() < FIELD_CORNER.getX() &&
                pose2.pose.getX() > 0.0 &&
                pose2.pose.getY() < FIELD_CORNER.getY() &&
                pose2.pose.getY() > 0.0 &&
                // pose2.tagCount >= 2 &&
                pose1.pose.getTranslation().getDistance(odometryPose.getTranslation()) < ALLOWABLE_POSE_DIFFERENCE &&
                pose2.avgTagDist < MAX_TAG_DISTANCE);

        if (pose1Trust && pose2Trust) {
            return ((pose1.avgTagDist < pose2.avgTagDist) ? pose1 : pose2); //select the limelight that has closer tags.
        } else if (pose1Trust) {
            return pose1;
        } else if (pose2Trust) {
            return pose2;
        } else
            return null;
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
        PoseEstimate pose1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(APRILTAG_LIMELIGHT2_NAME);
        PoseEstimate pose2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(APRILTAG_LIMELIGHT3_NAME);

        Boolean pose1Valid = (pose1.pose.getX() < FIELD_CORNER.getX() &&
                pose1.pose.getX() > 0.0 &&
                pose1.pose.getY() < FIELD_CORNER.getY() &&
                pose1.pose.getY() > 0.0);
        Boolean pose2Valid = (pose2.pose.getX() < FIELD_CORNER.getX() &&
                pose2.pose.getX() > 0.0 &&
                pose2.pose.getY() < FIELD_CORNER.getY() &&
                pose2.pose.getY() > 0.0);

        if (pose1Valid && pose2Valid) {
            return ((pose1.avgTagDist < pose2.avgTagDist) ? pose1 : pose2);
        } else if (pose1Valid) {
            return pose1;
        } else if (pose2Valid) {
            return pose2;
        } else
            return null;
    }

    public LimelightDetectorData getNeuralDetectorValues() {
        return new LimelightDetectorData(
                LimelightHelpers.getTX(OBJ_DETECITON_LIMELIGHT_NAME),
                LimelightHelpers.getTY(OBJ_DETECITON_LIMELIGHT_NAME),
                LimelightHelpers.getTA(OBJ_DETECITON_LIMELIGHT_NAME),
                LimelightHelpers.getNeuralClassID(OBJ_DETECITON_LIMELIGHT_NAME),
                LimelightHelpers.getTV(OBJ_DETECITON_LIMELIGHT_NAME));
    }
}