package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkMax intake = new SparkMax(Constants.MotorIDs.intakeID, SparkLowLevel.MotorType.kBrushless);
    public Intake(){}
    public void setSpeed(double speed){
        intake.set(speed);
    }
    public void stopIntake(){
        intake.set(0);
    }
}
