// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;

/**Contains the data from the Limelight Neural network object detection pipeline*/
public class LimelightDetectorData {

    public boolean isResultValid;
    /**Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
    public double tx;
    /**Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
    public double ty;
    /**Target Area (0% of image to 100% of image) */
    public double ta;
    
    public String classID;

    public LimelightDetectorData(double tx, double ty, double ta, String classID, boolean valid) {
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
        this.classID = classID;
        this.isResultValid = valid;
    }
}
