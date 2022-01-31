// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera extends SubsystemBase {
   NetworkTable table = NetworkTableInstance.getDefault().getTable("key");
  double targetFound = 0;
  double x = 0;
  double y = 0;
  double area = 0;
  
  public Camera() {}

  @Override
  public void periodic() {
    targetFound = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("Target Found", targetFound);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public double getTargetFound() {
    // return tv.getDouble(0.0);
    return targetFound;
  }

  public double getYOffset() {
    return y;
  }

  public double getXOffset() {
    return x;
  }
}