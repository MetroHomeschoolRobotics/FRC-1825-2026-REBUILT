package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//???????????????????????????? Who knows, not me :(
public class Indexer extends SubsystemBase{
    private TalonFX belt = new TalonFX(Constants.MotorIDs.beltID);
    private TalonFX indexer = new TalonFX(Constants.MotorIDs.indexerID);
    public Indexer(){}

    public void setIndexerSpeed(double speed){
        indexer.set(speed);
    }
    public void setBeltSpeed(double speed){
        belt.set(speed);
    }
    public void stopIndexer(){
        indexer.set(0);
    }
    public void stopBelt(){
        belt.set(0);
    }

}
