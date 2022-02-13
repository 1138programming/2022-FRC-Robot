package frc.robot.commands.Storage;

import frc.robot.Robot;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

public class TopStorageIn extends CommandBase {
    private Storage storage;

    public TopStorageIn(Storage storage){
        this.storage = storage;
        addRequirements(storage);
    }

    public void intialize(){}

    public void excute(){
        storage.move(kStorage,0);
    }

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return false;
    }
}
