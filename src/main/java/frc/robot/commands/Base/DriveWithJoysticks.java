// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.NeoBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithJoysticks extends CommandBase {

  private final NeoBase base;

  private double fbSpeed; //Speed of the robot in the x direction (forward).
  private double lrSpeed; //Speed of the robot in the Y direction (sideways).
  private double rot;

  private SlewRateLimiter xSpeedLimiter;
  private SlewRateLimiter ySpeedLimiter;
  private SlewRateLimiter rotLimiter;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(NeoBase base) {

    this.base = base;
  
    xSpeedLimiter = new SlewRateLimiter(4);
    ySpeedLimiter = new SlewRateLimiter(4);
    rotLimiter = new SlewRateLimiter(4);

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // base.resetAllRelEncoders();  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fbSpeed = xSpeedLimiter.calculate(Robot.robotContainer.getLogiLeftYAxis());
    fbSpeed = (Robot.robotContainer.getLogiLeftYAxis());
    
    // lrSpeed = ySpeedLimiter.calculate(Robot.robotContainer.getLogiLeftXAxis());
    lrSpeed = (Robot.robotContainer.getLogiLeftXAxis());
    
    // rot = rotLimiter.calculate(Robot.robotContainer.getLogiRightXAxis());
    rot = (Robot.robotContainer.getLogiRightXAxis());
    
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
