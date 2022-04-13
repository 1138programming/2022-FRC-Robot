// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//wpilib
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

    // Creates UsbCamera and connects them (only this one line is necessary for usb camera to show up on shuffleboard)
    // CameraServer.startAutomaticCapture(0);

  }
  
  @Override
  public void periodic() {
    //getting limelight networktable values
    targetFound = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    SmartDashboard.putBoolean("Limelight Target Found", getTargetFound());
    
    SmartDashboard.putNumber("Distance to Hub", getDistance());
  }

  public void LEDOn() {
    //Eye Protection
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //(turns limelight on)
  }
  public void LEDOff() {
    //Eye Protection
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //(turns limelight off)
  }
  public void LEDBlink() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //(blinks limelight)
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
      return distance + 8; //constant offset for this specific bot
    }
    else {
      //default value to return when limelight does not see target
      return kDistanceWhenNoTarget;
    }
  }
}