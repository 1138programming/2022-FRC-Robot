// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;

public class FlywheelAutoSpinUp extends CommandBase {
  private final Flywheel flywheel;
  private final Camera camera;
  private final Storage storage;
  private double flywheelOutput;
  private double encoderUnitOutput;
  private double distanceFromHub;
  /** Creates a new FlywheelAutoSpinUp. */
  public FlywheelAutoSpinUp(Flywheel flywheel, Camera camera, Storage storage) {
    this.flywheel = flywheel;
    this.camera = camera;
    this.storage = storage;
    flywheelOutput = 0;
    distanceFromHub = 0;
    // ballInBottomStorage = Robot.robotContainer.getStorageBottomSensor();
    // ballInTopStorage = Robot.robotContainer.getStorageTopSensor();
    addRequirements(flywheel);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("manual dist", 0);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ballInBottomStorage = Robot.robotContainer.getStorageBottomSensor();
    // ballInTopStorage = Robot.robotContainer.getStorageTopSensor();
    //nolan testing data: 202 in. is 85% (2602 rpm), 60 in.(closest we can get to hub) is 65% flywheel speed (1950 rpm)
    //assume linear relationship between distance and RPM required to score, (2462-1883)/(202-60) = 4.077
    distanceFromHub = camera.getDistance();
    // distanceFromHub = SmartDashboard.getNumber("manual dist", 0);

    if (distanceFromHub > 60) {
      if (distanceFromHub < 95) {
        flywheelOutput = 1700 + distanceFromHub * 4.077;
      }
      else if (distanceFromHub < 100) {
        flywheelOutput = 1350 + distanceFromHub * 4.077;
      }
      else if (distanceFromHub < 150) {
        flywheelOutput = 2200 + distanceFromHub * 4.077;
      }
      else {
        flywheelOutput = 2450 + distanceFromHub * 4.077;
      }
    }
    else {
      flywheelOutput = 0;
    }
    SmartDashboard.putNumber("flywheel output (RPM)", flywheelOutput);
    double flywheelOutputPercent = RobotContainer.scaleBetween(flywheelOutput, -1, 1, -kFlywheelMaxRPM, kFlywheelMaxRPM);
    // SmartDashboard.putNumber("flywheel output (percent)", flywheelOutput);
    // encoderUnitOutput = flywheelOutput * (512.0 / 75.0);
    // SmartDashboard.putNumber("flywheel output (u/100ms)", encoderUnitOutput);
    // SmartDashboard.putNumber("flywheel output (Percent)", flywheelOutput);
    // if (storage.getBallSensorMid() || storage.getBallSensorTop()) {
    if (storage.getBallSensorTop()) {
      flywheel.move(flywheelOutputPercent); 
      // flywheel.move(KFlywheelSpeed); 
    }
    else {
      flywheel.move(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
