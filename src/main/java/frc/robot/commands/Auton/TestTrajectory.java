// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import frc.robot.subsystems.NeoBase;

public class TestTrajectory extends CommandBase {
  TrajectoryConfig config;
  Trajectory trajectory;
  NeoBase base;
  SwerveControllerCommand command;
  int stage;
  

  public TestTrajectory(NeoBase base) {
    this.base = base;
    config = new TrajectoryConfig(0.3, 0.3);
    config.setKinematics(base.getKinematics());
    SmartDashboard.putString("Kinematics", base.getKinematics().toString());
    // trajectory = TrajectoryGenerator.generateTrajectorPy
    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(-1, 0), new Translation2d(0,0)),
      new Pose2d(0, 0, new Rotation2d(0)),
      config
    ); 
    Trajectory test1 = PathPlanner.loadPath("Test1", 0.5, 0.5);
    stage = 1;
    // command = new SwerveControllerCommand
    command = new SwerveControllerCommand(
      trajectory, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      new ProfiledPIDController (1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI)),
      base::setModuleStates,
      base
    );
    addRequirements(base);
    base.resetOdometry(trajectory.getInitialPose());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // base.resetWheels();
    base.resetAllRelEncoders();
    base.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, false);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (command.isFinished()) {
      SmartDashboard.putBoolean("isFinished()", true);
      return true;
    }
    SmartDashboard.putBoolean("isFinished()", false);
    return false;
  }
}
