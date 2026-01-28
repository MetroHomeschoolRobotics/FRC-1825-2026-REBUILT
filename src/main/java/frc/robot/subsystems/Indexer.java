package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//???????????????????????????? Who knows, not me :(
public class Indexer extends SubsystemBase{
    private SparkMax indexer = new SparkMax(Constants.MotorIDs.IndexerID, SparkLowLevel.MotorType.kBrushless);
    public Indexer(){}

    public void setSpeed(double speed){
        indexer.set(speed);
    }
    public void stopIndexer(){
        indexer.set(0);
    }

}
