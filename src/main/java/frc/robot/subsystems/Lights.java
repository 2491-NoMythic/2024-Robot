// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;import edu.wpi.first.wpilibj2.command.SubsystemBase;
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends SubsystemBase {
  /** Creates a new SubsystemLights. */
  private AddressableLED lights;
  private AddressableLEDBuffer LEDBuffer;
  int shooterLightsStart = 20;
  int shooterLightsEnd = 40;
  int shooterRailLightsStart = 0;
  int shooterRailLightsEnd = 19;
  int OtherLightsStart = 41;
  int OtherLightsEnd = 60;
  
  public Lights() {
    lights = new AddressableLED(6);
    LEDBuffer = new AddressableLEDBuffer(60);
    lights.setLength(60);
  }
  public void dataSetter(){
    lights.setData(LEDBuffer);
    lights.start();
  }
  public void setOneLightRGB(int index, int R, int G, int B){
    LEDBuffer.setRGB(index, R, G, B);
  }
  public void setLights(int start, int end, int R, int G, int B){
    for(int i = start; i < end; i++){
      setOneLightRGB(i, R, G, B);
    }
  }
  //All setLights methods are based on having four rows of fifteen lights, divided into three sections. 
  public void setLeft(int R, int G, int B){
    setLights(10, 20, R, G, B);
    setLights(40, 50, R, G, B);
  }

  public void setMid(int R, int G, int B){
   setLights(5, 10, R, G, B); 
   setLights(20, 25, R, G, B); 
   setLights(35, 40, R, G, B); 
   setLights(50, 55, R, G, B); 
  }

  public void setRight(int R, int G, int B){
    setLights(0, 5, R, G, B);
    setLights(25, 35, R, G, B);
    setLights(55, 60, R, G, B);
  }

  /**
   * Set one light in a horizontal line on all 4 strips
   * @param pos 0-14 where 0 is the far left LED and 14 is the far right LED
   */
  public void setHorizontal(int pos, int R, int G, int B){
    setOneLightRGB((14 - pos), R, G, B);
    setOneLightRGB((15 + pos), R, G, B);
    setOneLightRGB((44 - pos), R, G, B);
    setOneLightRGB((45 + pos), R, G, B);
  }

  /**
   * Set each side to 2 seperate colors based on a given position
   * 11122-----22111
   *    ^pos
   * @param pos the number of lights to the left of the split
   */
  public void setSplit(int pos, int R1, int G1, int B1, int R2, int G2, int B2){
    for(int i = 0; i < pos; i++){
      setHorizontal(i, R1, G1, B1);
      setHorizontal(14 - i, R1, G1, B1);
    }
    for(int i = pos; i < 5; i++){
      setHorizontal(i, R2, G2, B2);
      setHorizontal(14 - i, R2, G2, B2);
    }

  }

  /**
   * set lights based on a distance from 0
   * when the absolute value of progress is greater than 1 all lights are color 1
   * when the absolute value of progress is near 0 all lights are color 2
   * when the absolute value of progress is in between 0 and 1 some lights will be color 1 and some will be color 2
   */
  public void setProgress(double progress, int R1, int G1, int B1, int R2, int G2, int B2){
    int count = (int)Math.abs(progress * 5);
    if(count > 5){
      count = 5;
    }

    if (progress < 0){
      setSplit(count, R1, G1, B1, R2, G2, B2);
    }
    else{
      setSplit(5 - count, R2, G2, B2, R1, G1, B1);
    }
  }

  public void setSides(int R, int G, int B){
    setLeft(R, G, B);
    setRight(R, G, B);
  }
  public void lightsOut() {
    setLights(0, LEDBuffer.getLength(), 0, 0, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dataSetter();
  }
}