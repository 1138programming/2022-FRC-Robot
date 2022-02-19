package frc.robot.commands.Storage;

import frc.robot.Robot;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

public class TopStorageOut extends CommandBase {
    private Storage storage;

    public TopStorageOut(Storage storage){
        this.storage = storage;
        addRequirements(storage);
    }

    public void intialize(){}
    
    public void execute(){
        //First number is the top motor, second number is the bottom motor
        storage.move(-kStoragePWM,0);
    }

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return false;
    }
}
