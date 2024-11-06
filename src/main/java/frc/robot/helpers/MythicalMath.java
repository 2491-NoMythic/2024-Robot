// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

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
    
}
