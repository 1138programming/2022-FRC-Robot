package frc.robot.commands.Hang;

import frc.robot.Robot;
import frc.robot.subsystems.Hang;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HangMove extends CommandBase {

    private final Hang hang;

    public HangMove(Hang hang) {
        this.hang = hang;
        addRequirements(hang);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.move(1, 1); // may have to change values
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