// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;

import frc.robot.Robot;
import frc.robot.Gains;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.NeoBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class DriveWithLimelight extends CommandBase {

  private double fbSpeed; //Speed of the robot in the x direction (forward).
  private double lrSpeed; //Speed of the robot in the Y direction (sideways).
  private double rot;
  private Camera camera;
  private NeoBase base;
  private PIDController rotationController;
  private Gains pidGains;

  /** Creates a new DriveWithLimelight. */
  public DriveWithLimelight(NeoBase base, Camera camera) {

    this.base = base;
    this.camera = camera;

    pidGains = new Gains(0.8, 0, 0, 0);
    rotationController = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);

    addRequirements(base);
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders(); 
    if (camera.getTargetFound() == 0) {
      SmartDashboard.putBoolean("Target Found", false);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.LEDOn();
    // fbSpeed = xSpeedLimiter.calculate(Robot.robotContainer.getLogiLeftYAxis());
    fbSpeed = (-Robot.robotContainer.getLogiLeftYAxis());
    
    // lrSpeed = ySpeedLimiter.calculate(Robot.robotContainer.getLogiLeftXAxis());
    lrSpeed = (Robot.robotContainer.getLogiLeftXAxis());
    
    // rot = rotLimiter.calculate(Robot.robotContainer.getLogiRightXAxis());
    rot = (-Robot.robotContainer.getLogiRightXAxis());

    rot -= rotationController.calculate(camera.getXOffset()/KLimelightRange, 0);
    
    base.drive(fbSpeed, lrSpeed, rot, false);
    
    SmartDashboard.putNumber("fbspeed", fbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
