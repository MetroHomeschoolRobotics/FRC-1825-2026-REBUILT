package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//???????????????????????????? Who knows, not me :(
public class Indexer extends SubsystemBase{
    private TalonFX indexer = new TalonFX(Constants.MotorIDs.IndexerID);
    public Indexer(){}

    public void setSpeed(double speed){
        indexer.set(speed);
    }
    public void stopIndexer(){
        indexer.set(0);
    }

}
