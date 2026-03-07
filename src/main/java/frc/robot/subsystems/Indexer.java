package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//???????????????????????????? Who knows, not me :(
public class Indexer extends SubsystemBase{
    private TalonFX belt = new TalonFX(Constants.MotorIDs.beltID,"*");
    private TalonFX indexer = new TalonFX(Constants.MotorIDs.indexerID,"*");
    private TalonFXConfiguration config = new TalonFXConfiguration();
    public Indexer(){
        setConfigs();
        belt.getConfigurator().apply(config);
        indexer.getConfigurator().apply(config);
    }

    public void setIndexerSpeed(double speed){
        indexer.set(speed);
    }
    private void setConfigs(){
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.MotorOutput.Inverted =InvertedValue.CounterClockwise_Positive;
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
