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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;

public class FlywheelSpinWithLimelight extends CommandBase {
  private final Flywheel flywheel;
  private final Camera camera;
  private double flywheelOutput;
  // private double encoderUnitOutput;
  private double distanceFromHub;
  /** Creates a new FlywheelSpinWithLimelight. */
  public FlywheelSpinWithLimelight(Flywheel flywheel, Camera camera) {
    this.flywheel = flywheel;
    this.camera = camera;
    flywheelOutput = 0;
    distanceFromHub = 0;
    addRequirements(flywheel);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //nolan testing data: 202 in. is 85% (2602 rpm), 60 in.(closest we can get to hub) is 65% flywheel speed (1950 rpm)
    //assume linear relationship between distance and RPM required to score, (2462-1883)/(202-60) = 4.077
    distanceFromHub = camera.getDistance();
    // distanceFromHub = SmartDashboard.getNumber("manual dist", 0);

    flywheelOutput = flywheel.calculateFlywheelSpeedFromDist(distanceFromHub);

    flywheel.move(flywheelOutput);
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
