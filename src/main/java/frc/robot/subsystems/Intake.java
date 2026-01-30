package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intake = new TalonFX(Constants.MotorIDs.intakeID);
    public Intake(){}
    public void setSpeed(double speed){
        intake.set(speed);
    }
    public void stopIntake(){
        intake.set(0);
    }
}
