// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

public class Camera extends SubsystemBase {
  NetworkTable table;
  double targetFound;
  double x;
  double y;
  double area;
  
  public Camera() {
    //setting up networktable
    table = NetworkTableInstance.getDefault().getTable("key");
    targetFound = 0;
    x = 0;
    y = 0;
    area = 0;    
  }
  
  @Override
  public void periodic() {
    //getting networktable values
    targetFound = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance to Hub", getDistance());
  }
  public void LEDOn() {
    //Eye Protection
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //(turns limelight off) For Testing only
  }
  public void LEDOff() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //(turns limelight on) For Testing only
  }
  public void LEDBlink() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //(blinks limelight ) For Testing only
  }
  
  public double getTargetFound() {
    return targetFound;
  }

  public double getYOffset() {
    return y;
  }

  public double getXOffset() {
    return x;
  }

  public double getDistance() {
    double distance = KHeightDifference / Math.tan(Math.toRadians(KLimelightAngle + y));
    return distance;
  }
}