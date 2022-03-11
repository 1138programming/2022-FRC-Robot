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
        //logik loop
        if (!ballInTop) {   // if the top sensor doesn't see a ball, both motors run
            // storage.move(kStoragePWM, kStoragePWM);
            storage.moveTop(kStoragePWM);
            storage.moveBottom(kStoragePWM);
        }
        else if (!ballInBottom) {   // if the bottom sensor doesn't see a ball, only the bottom motor will run
            storage.moveTop(0);
            storage.moveBottom(kStoragePWM);
        }
        else {  // if both sensors see a ball, neither motor will run
            storage.moveBottom(0);
            storage.moveTop(0);
        }
    }

    public void end(boolean interrupted){
        storage.moveTop(0);
        storage.moveBottom(0);
    }

    //stops return value
    public boolean isFinished(){
        return false;
    }
}
