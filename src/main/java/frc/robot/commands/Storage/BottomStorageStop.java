package frc.robot.commands.Storage;

import frc.robot.Robot;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

public class BottomStorageStop extends CommandBase {
    private Storage storage;

    public BottomStorageStop(Storage storage){
        this.storage = storage;
        
        addRequirements(storage);
    }

    public void intialize(){

    }

    public void execute(){
        storage.moveBottom(0);
    }

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return false;
    }
}
