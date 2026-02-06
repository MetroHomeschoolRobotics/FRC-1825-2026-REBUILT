package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intake = new TalonFX(Constants.MotorIDs.intakeID);
    private TalonFX intakeRetractor = new TalonFX(Constants.MotorIDs.intakeRetractorID);
    public Intake(){}
    public void setIntakeSpeed(double speed){
        intake.set(speed);
    }
    public void stopIntake(){
        intake.set(0);
    }
    public void setRetractorSpeed(double speed){
        intakeRetractor.set(speed);
    }
    public void stopIntakeRetractor(){
        intakeRetractor.set(0);
    }
}
