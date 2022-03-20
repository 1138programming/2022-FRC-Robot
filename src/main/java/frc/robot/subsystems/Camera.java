// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import static frc.robot.Constants.*;

public class Camera extends SubsystemBase {
  private NetworkTable table;
  private double targetFound;
  private double x;
  private double y;
  private double area;
  
  public Camera() {
    //setting up networktable
    table = NetworkTableInstance.getDefault().getTable("key");
    targetFound = 0;
    x = 0;
    y = 0;
    area = 0;

  // Creates UsbCamera and MjpegServer [1] and connects them
  CameraServer.startAutomaticCapture(0);

  // Creates the CvSink and connects it to the UsbCamera
  CvSink cvSink = CameraServer.getVideo();

  // Creates the CvSource and MjpegServer [2] and connects them
  CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

  }
  
  @Override
  public void periodic() {
    //getting networktable values
    targetFound = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    SmartDashboard.putBoolean("Limelight Target Found", getTargetFound());
    
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance to Hub", getDistance());
  }
  public void LEDOn() {
    //Eye Protection
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //(turns limelight on) For Testing only
  }
  public void LEDOff() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //(turns limelight off) For Testing only
  }
  public void LEDBlink() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //(blinks limelight) For Testing only
  }
  
  public boolean getTargetFound() {
    if (targetFound == 0)
    {
      return false;
    }
    else if (targetFound == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getYOffset() {
    return y;
  }

  public double getXOffset() {
    return x;
  }

  public double getDistance() {
    if (getTargetFound()) {
      double distance = KHeightDifference / Math.tan(Math.toRadians(KLimelightAngle + y));
      return distance + 8; //constant offset
    }
    else {
      return 46.0;
    }
  }
}