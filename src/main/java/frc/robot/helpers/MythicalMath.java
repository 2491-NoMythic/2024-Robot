// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import java.security.PublicKey;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class MythicalMath {
/**
 * finds the absolute distance from the origin of any point on a 3d plane
 * @param XfromOrigin x-coordinate of point
 * @param YfromOrigin y-coordinate of point
 * @param ZfromOrigin z-coordinate of point
 * @return distance from origin of point
 *
 */
    public static double DistanceFromOrigin3d(double XfromOrigin, double YfromOrigin, double ZfromOrigin) {
        double Distance2d = Math.sqrt(Math.pow(YfromOrigin, 2)+Math.pow(XfromOrigin, 2));
        return Math.sqrt(Math.pow(Distance2d, 2)+Math.pow(ZfromOrigin, 2));
    }

    /**
     *  finds the absolute distance from the origin of any point on a 3d plane
     * @param pose the pose you want to find the distnace of
     * @return distance from origin of point
     */
    public static double DistanceFromOrigin3d(Pose3d pose) {
        return DistanceFromOrigin3d(pose.getX(),pose.getY(), pose.getZ());
    }

    
    /**
     *  finds the absolute distance from the origin of any point on a 3d plane
     * @param pose the pose you want to find the distnace of
     * @return distance from origin of point
     */
    public static double DistanceFromOrigin3d(Pose2d pose) {
        return DistanceFromOrigin3d(pose.getX(),pose.getY(),0);
    }
 /**
     *  finds the absolute distance from the origin of any point on a 3d plane
     * @param vector the vector you want to find the distnace of
     * @return distance from origin of point
     */
    public static double DistanceFromOrigin3d(mythicalVector3 vector) {
        return DistanceFromOrigin3d(vector.x ,vector.y,vector.z);
    }


    public static Pose2d multiplyOnlyPos(Pose2d pose, double scalar)
    {
        return new Pose2d(pose.getX()*scalar,pose.getY()*scalar,pose.getRotation());
    }

    public static Pose2d divideOnlyPos(Pose2d pose, double scalar)
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



    /**
     * This is a class to simply hold 3 numbers, in a clean conssistent way, without using pose3Ds, as pose3Ds use rotations
     */
    public static class mythicalVector3{
        double x;
        double y;
        double z;

        public mythicalVector3(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public mythicalVector3(double value)
        {
            this.x = value;
            this.y = value;
            this.z = value;
        }

        
        public mythicalVector3(Pose3d pose)
        {
            this.x = pose.getX();
            this.y = pose.getY();
            this.z = pose.getZ();
        }

        
          public mythicalVector3(Pose2d pose)
        {
            this.x = pose.getX();
            this.y = pose.getY();
            this.z = 0;
        }


        /**
         * @param scalar the number applied to all 3 numbers
         * @return
         */
        public mythicalVector3 div(double scalar)
        {
            return new mythicalVector3(this.x/scalar, this.y/scalar, this.z/scalar);
        }

        /**
         * @param vector the vector you want to affect your this vector by
         * @return
         */
        public mythicalVector3 div(mythicalVector3 vector)
        {
            return new mythicalVector3(this.x/vector.x, this.y/vector.y, this.z/vector.z);
        }

        /**
         * @param scalar the number applied to all 3 numbers
         * @return
         */
        public mythicalVector3 times(double scalar)
        {
            return new mythicalVector3(this.x*scalar, this.y*scalar, this.z*scalar);
        }

           /**
         * @param vector the vector you want to affect your this vector by
         * @return
         */
        public mythicalVector3 times(mythicalVector3 vector)
        {
            return new mythicalVector3(this.x*vector.x, this.y*vector.y, this.z*vector.z);
        }

        /**
         * @param scalar the number applied to all 3 numbers
         * @return
         */
        public mythicalVector3 plus(double scalar)
        {
            return new mythicalVector3(this.x+scalar, this.y+scalar, this.z+scalar);
        }

           /**
         * @param vector the vector you want to affect your this vector by
         * @return
         */
        public mythicalVector3 plus(mythicalVector3 vector)
        {
            return new mythicalVector3(this.x+vector.x, this.y+vector.y, this.z+vector.z);
        }

          /**
         * @param scalar the number applied to all 3 numbers
         * @return
         */
        public mythicalVector3 minus(double scalar)
        {
            return new mythicalVector3(this.x-scalar, this.y-scalar, this.z-scalar);
        }

          /**
         * @param vector the vector you want to affect your this vector by
         * @return
         */
        public mythicalVector3 minus(mythicalVector3 vector)
        {
            return new mythicalVector3(this.x-vector.x, this.y-vector.y, this.z-vector.z);
        }

        public mythicalVector3 abs()
        {
            double x = this.x;
            double y = this.y;
            double z = this.z;

            if (this.x<0 ) { x=this.x*-1;}
             if (this.y<0 ) { y=this.y*-1;}
              if (this.z<0 ) { z=this.z*-1;}

              return new mythicalVector3(x, y, z);
        }

        public boolean isLargerThan(mythicalVector3 vector)
        {
            if(vector.x > this.x){
                return true;
            }
            else if (vector.y > this.y){
                return true;
            }
            else if (vector.z > this.z){
                return true;
            }
            else{
                return false;
            }
        }

        public boolean isLargerThan(double value)
        {
          return isLargerThan(new mythicalVector3(value));
        }

          public boolean isSmallerThan(mythicalVector3 vector)
        {
            if(vector.x < this.x){
                return true;
            }
            else if (vector.y < this.y){
                return true;
            }
            else if (vector.z < this.z){
                return true;
            }
            else{
                return false;
            }
        }

        public boolean isSmallerThan(double value)
        {
          return isSmallerThan(new mythicalVector3(value));
        }

        //CONVERSIONS

        /**
         * 
         * @param rotation the rotation you want the pose 3D to have
         * @return a pose 3d 
         */
        public Pose3d toPose3d (Rotation3d rotation)
        {
            return new Pose3d(x,y,z,rotation);
        }


        /**
         *@return a pose 3d, with the same position, with all rotations = 0.
         */
        public Pose3d toPose3d ()
        {
            return toPose3d(new Rotation3d(0,0,0));
        }

        public static mythicalVector3 pose3dToVector(Pose3d pose)
        {
        return new mythicalVector3(pose.getX(), pose.getY(), pose.getZ());
        }

    }
    
}
