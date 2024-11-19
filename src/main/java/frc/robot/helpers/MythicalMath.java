// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import java.security.PublicKey;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class MythicalMath {
/**
 * finds the absolute distance from the origin of any point on a 3d plane
 * @param XfromOrigin x-coordinate of point
 * @param YfromOrigin y-coordinate of point
 * @param ZfromOrigin z-coordinate of point
 * @return distance from origin of point
 */
    public static double DistanceFromOrigin3d(double XfromOrigin, double YfromOrigin, double ZfromOrigin) {
        double Distance2d = Math.sqrt(Math.pow(YfromOrigin, 2)+Math.pow(XfromOrigin, 2));
        return Math.sqrt(Math.pow(Distance2d, 2)+Math.pow(ZfromOrigin, 2));
    }

    public static Pose2d multiplyOnlyPos(Pose2d pose, Double scalar)
    {
        return new Pose2d(pose.getX()*scalar,pose.getY()*scalar,pose.getRotation());
    }

    public static Pose2d divideOnlyPos(Pose2d pose, Double scalar)
    {
        return new Pose2d(pose.getX()/scalar, pose.getY()/scalar,pose.getRotation());
    }
    
    /**
     * 
     * @param pose1
     * @param pose2
     * @return  poses added together, with pose1's rotation
     */
    public static Pose2d addOnlyPosTogether(Pose2d pose1, Pose2d pose2)
    {
     return new Pose2d(pose1.getX()+pose2.getX(), pose1.getY()+pose2.getY() ,pose1.getRotation());
    }
}
