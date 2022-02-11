package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class SwerveTrajectory {
  public SwerveTrajectory() {

  }
  private HolonomicDriveController HDC = new HolonomicDriveController(
    new PIDController(0.047116, 0, 0), 
    new PIDController(0.047116, 0, 0), 
    new ProfiledPIDController(0.69, 0, 0, new TrapezoidProfile.Constraints(8, 5)));
    Trajectory test1 = PathPlanner.loadPath("Test1", 8, 5);

  
}
