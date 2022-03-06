package frc.robot.commands.Storage;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

//Constructor
public class StorageCollect extends CommandBase {
    private Storage storage;

    public StorageCollect(Storage storage){
        this.storage = storage;
        addRequirements(storage);
    }

    public void intialize(){}

    public void execute(){
        boolean ballInTop = storage.getBallSensorTop();
        boolean ballInBottom = storage.getBallSensorBottom();
        //start by moving the bottom motor
        storage.move(0, 1);
        //logik loop
        
        if (!ballInTop) {   // if the top sensor doesn't see a ball, both motors run
            storage.move(0, 0);
        }
        else if (!ballInBottom) {   // if the bottom sensor doesn't see a ball, only the bottom motor will run
            storage.move(0, 0.5);
        }
        else {  // if both sensors see a ball, neither motor will run
            storage.move(0, 0);
        }
    }

    public void end(boolean interrupted){}

    //stops return value
    public boolean isFinished(){
        return false;
    }
}
