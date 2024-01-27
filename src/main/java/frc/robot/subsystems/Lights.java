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
  
  public Lights(int LedCount) {
    lights = new AddressableLED(6);
    LEDBuffer = new AddressableLEDBuffer(LedCount);
    lights.setLength(LedCount);
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
  public void setLightsCone() {
    for(int i = 0; i < 52; i++){
      setOneLightRGB(i, 100, 64, 0);
  }
}
  public void setLightsCube() {
    for(int i = 0; i < 52; i++){
      setOneLightRGB(i, 0, 0, 100);
  }
  }
  public void lightsOut() {
    for(int i = 0; i < LEDBuffer.getLength(); i++) {
      setOneLightRGB(i, 0, 0, 0);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dataSetter();
  }
}