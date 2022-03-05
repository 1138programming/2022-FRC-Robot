// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import javax.crypto.interfaces.DHPublicKey;

import edu.wpi.first.math.controller.PIDController;
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

  double rotP;
  double rotI;
  double rotD;

  private PIDController fbController;
  private PIDController lrController;
  private PIDController rotController;

  /** Creates a new DriveToPose. */
  public DriveToPose(NeoBase base, Pose2d targetPose) {
    this.base = base;
    // this.targetPose = targetPose;

    this.targetPose = new Pose2d(SmartDashboard.getNumber("new x", 0), SmartDashboard.getNumber("new y", 0), new Rotation2d());
    SmartDashboard.putString("targetPose", this.targetPose.toString());
    

    currentPose = base.getPose();

    fbController = new PIDController(1, 0, 0);
    lrController = new PIDController(1, 0, 0);
    rotController = new PIDController(SmartDashboard.getNumber("rotP", 0), SmartDashboard.getNumber("rotI", 0), SmartDashboard.getNumber("rotD", 0));
    

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = new Pose2d(SmartDashboard.getNumber("new x", 0), SmartDashboard.getNumber("new y", 0), Rotation2d.fromDegrees(SmartDashboard.getNumber("new rotation", 0)));
    SmartDashboard.putString("targetPose", targetPose.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = base.getPose();
    fbOffset = targetPose.getX() - currentPose.getX();
    lrOffset = targetPose.getY() - currentPose.getY();
    headingOffset = base.getHeadingDeg() - currentPose.getRotation().getDegrees();

    fbSpeed = fbController.calculate(currentPose.getX(), targetPose.getX());
    lrSpeed = lrController.calculate(currentPose.getY(), targetPose.getY());

    rotSpeed = rotController.calculate(base.getHeadingDeg(), targetPose.getRotation().getDegrees());

    base.drive(-fbSpeed, lrSpeed, rotSpeed, true);

    SmartDashboard.putNumber("fb offset", fbOffset);
    SmartDashboard.putNumber("lr offset", lrOffset);
    SmartDashboard.putNumber("fb speed", fbSpeed);
    SmartDashboard.putNumber("lr speed", lrSpeed);
    SmartDashboard.putNumber("rotSpeed", rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(fbOffset) < 0.1 && Math.abs(lrOffset) < 0.1 && Math.abs(headingOffset) < 0.1;
  }
}
