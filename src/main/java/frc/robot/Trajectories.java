package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.NeoBase;

public class Trajectories {
  private static double maxSpeedConfig = 2;
  private static double maxAccelConfig = 2;
  private static PIDController xController = new PIDController(1, 0, 0);
  private static PIDController yController = new PIDController(1, 0, 0);
  private static NeoBase base = new NeoBase();

  private static ProfiledPIDController thetaController = new ProfiledPIDController(
    0, 0, 0, new TrapezoidProfile.Constraints(1, 1));

  public static TrajectoryConfig config = new TrajectoryConfig(maxSpeedConfig, maxAccelConfig);;

  public static void enableContinuousInput() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  public static class Blue1 {
    Trajectory part1 = PathPlanner.loadPath("Blue 1 Part 1", maxSpeedConfig, maxAccelConfig);
  }

  public static class Red1 {
    private static Trajectory part1 = PathPlanner.loadPath("Red 1 Part 1", maxSpeedConfig, maxAccelConfig);
    private static SwerveControllerCommand part1Command = new SwerveControllerCommand(
      part1, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(0.8, 0, 0),
      new PIDController(0.8, 0, 0),
      thetaController,
      base::getHeading,
      base::setModuleStates,
      base
    );

    private static Trajectory part2 = PathPlanner.loadPath("Red 1 Part 2", maxSpeedConfig, maxAccelConfig);
    private static SwerveControllerCommand part2Command = new SwerveControllerCommand(
      part2, 
      base::getPose, 
      base.getKinematics(), 
      new PIDController(0.8, 0, 0),
      new PIDController(0.8, 0, 0),
      thetaController,
      base::getHeading,
      base::setModuleStates,
      base
    );

    public static void configTrajectories (NeoBase base) {
      
    }

    public static SwerveControllerCommand getTrajectory(int partNum) {
      switch (partNum) {
        case(1): 
          return part1Command;
        case(2): 
          return part2Command;
      }
      return part1Command;
    }
  }
}
