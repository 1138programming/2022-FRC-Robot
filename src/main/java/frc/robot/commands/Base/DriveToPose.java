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

  private double rotP = 0.05;
  private double rotI = 0;
  private double rotD = 0;

  private PIDController fbController;
  private PIDController lrController;
  private PIDController rotController;

  private SlewRateLimiter fbSpeedLimiter;
  private SlewRateLimiter lrSpeedLimiter;
  private SlewRateLimiter rotSpeedLimiter;

  /** Creates a new DriveToPose. */
  public DriveToPose(NeoBase base, Pose2d targetPose) {
    this.base = base;
    this.targetPose = targetPose;

    // this.targetPose = new Pose2d(SmartDashboard.getNumber("new x", 0), SmartDashboard.getNumber("new y", 0), new Rotation2d());
    SmartDashboard.putString("targetPose", this.targetPose.toString());
    

    currentPose = base.getPose();

    fbController = new PIDController(1, 0, 0);
    lrController = new PIDController(1, 0, 0);
    rotController = new PIDController(rotP, rotI, rotD);

    fbSpeedLimiter = new SlewRateLimiter(2);
    lrSpeedLimiter = new SlewRateLimiter(2);
    rotSpeedLimiter = new SlewRateLimiter(2);

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("rotP", rotP);
    SmartDashboard.putNumber("rotI", rotI);
    SmartDashboard.putNumber("rotD", rotD);

    SmartDashboard.putString("targetPose", targetPose.toString());
  }

  @Override
  public void execute() {

    rotController.setPID(SmartDashboard.getNumber("rotP", 0), SmartDashboard.getNumber("rotI", 0), SmartDashboard.getNumber("rotD", 0));

    currentPose = base.getPose();
    fbOffset = targetPose.getX() - currentPose.getX();
    lrOffset = targetPose.getY() - currentPose.getY();
    headingOffset = base.getHeadingDeg() - currentPose.getRotation().getDegrees();

    fbSpeed = fbController.calculate(currentPose.getX(), targetPose.getX());
    lrSpeed = lrController.calculate(currentPose.getY(), targetPose.getY());

    rotSpeed = rotController.calculate(base.getHeadingDeg(), targetPose.getRotation().getDegrees());

    // fbSpeed = MathUtil.clamp(fbSpeed, -2, 2);
    // lrSpeed = MathUtil.clamp(lrSpeed, -2, 2);
    rotSpeed = MathUtil.clamp(rotSpeed, -2, 2);

    base.drive(-fbSpeed, -lrSpeed, rotSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return Math.abs(fbOffset) < 0.05 && Math.abs(lrOffset) < 0.05 && Math.abs(headingOffset) < 0.05;
  }
}
