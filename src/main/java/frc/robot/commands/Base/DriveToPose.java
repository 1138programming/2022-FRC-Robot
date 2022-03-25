// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoBase;

public class DriveToPose extends CommandBase {
  private NeoBase base;
  private Pose2d currentPose;
  private Pose2d targetPose;

  private double headingOffset;
  private double lrOffset;
  private double fbOffset;

  private double fbSpeed;
  private double lrSpeed;
  private double rotSpeed;

  private double rotP = 4.5;
  private double rotI = 0;
  private double rotD = 0;

  private PIDController fbController;
  private PIDController lrController;
  private PIDController rotController;

  private SlewRateLimiter fbSpeedLimiter;
  private SlewRateLimiter lrSpeedLimiter;

  /** Creates a new DriveToPose. */
  public DriveToPose(NeoBase base, Pose2d targetPose) {
    this.base = base;
    this.targetPose = targetPose;

    currentPose = base.getPose();

    fbController = new PIDController(0.85, 0, 0);
    lrController = new PIDController(0.85, 0, 0);
    rotController = new PIDController(rotP, rotI, rotD);

    fbSpeedLimiter = new SlewRateLimiter(2);
    lrSpeedLimiter = new SlewRateLimiter(2);

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();  
  }

  @Override
  public void execute() {

    // rotController.setPID(SmartDashboard.getNumber("rotP", 0), SmartDashboard.getNumber("rotI", 0), SmartDashboard.getNumber("rotD", 0));

    currentPose = base.getPose();
    fbOffset = targetPose.getX() - currentPose.getX();
    lrOffset = targetPose.getY() - currentPose.getY();
    headingOffset = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

    fbSpeed = fbController.calculate(currentPose.getX(), targetPose.getX());
    lrSpeed = lrController.calculate(currentPose.getY(), targetPose.getY());

    fbSpeed = fbSpeedLimiter.calculate(fbSpeed);
    lrSpeed = lrSpeedLimiter.calculate(lrSpeed);
    rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees()/360, targetPose.getRotation().getDegrees()/360);
    
    base.drive(-fbSpeed, -lrSpeed, rotSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(fbOffset) < 0.06 && Math.abs(lrOffset) < 0.06 && Math.abs(headingOffset) < 0.1;
  }
}
