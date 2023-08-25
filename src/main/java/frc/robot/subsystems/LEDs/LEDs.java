// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  AHRS gyro;

  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int m_rainbowFirstPixelHue = 0;
  
  int mode = 3;
  /* 
  0 = Solid Color
  1 = Rainbow
  2 = Noise
  3 = Directional
  */

  OpenSimplex noise;
  double height = 20;
  double width = 600;
  double yscale = 0.05;
  double offset = 0.01;
  double offsetInc = 0.01;
  double[] hueRange = {85, 150};
  // double[] hueRange = {60, 150};
  //double[] hueRange = {150, 180+20};
  double[] noiseRange = {-1,1};

  Timer timer = new Timer();

  int hue = 80;
  int value = 128;

  /** Creates a new LEDs. */
  public LEDs(AHRS gyro) {
    this.gyro = gyro;

    m_led = new AddressableLED(6);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(108);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    noise = new OpenSimplex();

    this.mode = 3;
    this.hue = 80;
  }

  public void SetModeSolid(int hue){
    this.mode = 0;
    this.hue = hue;
    this.value = 128;
  }

  public void SetLEDsValue(int value){
    this.value = value;
  }

  public void SetModeRainbow(){
    this.mode = 1;
    this.value = 128;
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 120, value);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void SetModeNoise(double[] hueRange){
    this.hueRange = hueRange;
    this.mode = 2;
    this.value = 128;
  }

  private void noise(){
    for (var x = 0; x < m_ledBuffer.getLength(); x++) {
      double noiseVal = noise.noise2_ImproveX(1934173838, x*yscale, offset);
      int hue = (int)Remap(noiseVal, noiseRange, hueRange) % 180;
      m_ledBuffer.setHSV(x, hue, 450, value);
    }
    offset += offsetInc;

    if(Math.abs(offset) > 10000){
      offset *= -1;
    }
  }

  private void directional1(){
    Math.sin(m_rainbowFirstPixelHue);
    int max = m_ledBuffer.getLength() * 4 / 3 + 20;
    int directionalLen = m_ledBuffer.getLength() * 4 / 9 - 5;
    int directionalStart = 17;

    int hue1 = 50;
    int hue2 = 150;

    float direction = gyro.getYaw();
    if (direction < 0){
      direction += 360;
    }

    int offset = (int)(max * (direction / 360));
    // System.out.println("Angle: " + direction);
    // System.out.println("Offset: " + offset);
    directionalLen += max;
    directionalLen -= offset;
    directionalLen %= max;

    for (var x = 0; x < max+directionalLen; x++) {
      int pos = x % max;
      if(x >= m_ledBuffer.getLength() && x < max){
        //nothing
      }
      else if(x >= directionalStart && x < directionalStart + directionalLen){
        m_ledBuffer.setHSV(pos, hue1, 255, value);
      }
      else{
        m_ledBuffer.setHSV(pos, hue2, 255, value);
      }
    }
  }

  private void directional(){
    int max = (m_ledBuffer.getLength() + 15) * 4 / 3;
    int hue1 = 100;
    int hue2 = 180;

    double direction = gyro.getYaw();
    direction = direction * Math.PI / 180;
  
    for (var x = 0; x < m_ledBuffer.getLength(); x++) {
      double angle = (2 * Math.PI * x / max) + direction;
      int hue = (int) (((Math.sin(angle) + 1)/2) * (hue2 - hue1) + hue1);
      m_ledBuffer.setHSV(x, hue, 255, value);
    }
  }

  private double Remap(double startVal, double[] startRange, double[] endRange){
    return (startVal-startRange[0]) * (endRange[1]-endRange[0]) / (startRange[1]- startRange[0]) + endRange[0];
  }
  
  private double Lerp(double start, double end, double lerpVal){
    return (end-start) * lerpVal + start;
  }

  @Override
  public void periodic() {
    if(mode == 0){
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, hue, 255, value);
      }
      m_led.setData(m_ledBuffer);
    }
    else if(mode == 1)
    {
      rainbow();
      m_led.setData(m_ledBuffer);
    }
    else if (mode == 2)
    {
      noise();
      m_led.setData(m_ledBuffer);
    }
    else if (mode == 3)
    {
      directional();
      m_led.setData(m_ledBuffer);
    }
    // This method will be called once per scheduler run
  }
}
