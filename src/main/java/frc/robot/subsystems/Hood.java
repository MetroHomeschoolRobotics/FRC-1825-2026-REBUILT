package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private TalonFX hood1 = new TalonFX(Constants.MotorIDs.hoodID1);
    private TalonFX hood2 = new TalonFX(Constants.MotorIDs.hoodID2);
    private PIDController pid = new PIDController(.001, 0, 0);
    public Hood(){}
    public void setSpeed(double speed){
        hood1.set(speed);
        hood2.set(speed);
    }
    public void setPID(double setPoint){
        pid.setSetpoint(setPoint);
    }
    public double getAngle(){
        return Constants.MathConstants.defaultHoodAngle-(hood1.getPosition().getValueAsDouble()*Constants.MathConstants.hoodRotationsPerDegree)
        ;
    }
    

    public void periodic(){
        double output = pid.calculate(hood1.getPosition().getValueAsDouble());
        setSpeed(output);
    }
}
