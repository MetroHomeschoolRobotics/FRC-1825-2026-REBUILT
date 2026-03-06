package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;

public class RunFullIndexing extends Command {
    private Indexer indexer;
    public RunFullIndexing(Indexer _indexer){
        indexer = _indexer;
        addRequirements(_indexer);
    }
    public void initialize(){

    }
    public void execute(){
        //TO/DO check rotation directions later
        indexer.setBeltSpeed(Constants.Setpoints.beltSpeed);
        indexer.setIndexerSpeed(Constants.Setpoints.indexerSpeed);
    }
    public void end(){
        indexer.stopBelt();
        indexer.stopIndexer();
    }
    public boolean isFinished(){
        return false;
    }
}
