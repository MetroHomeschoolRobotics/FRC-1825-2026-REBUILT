package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase{
    private PIDController pid = new PIDController(0.01,0,0);
    
    private double desiredVelocity = 0;

    private TalonFX shooter1 = new TalonFX(Constants.MotorIDs.shooterMotorID1);
    private TalonFX shooter2 = new TalonFX(Constants.MotorIDs.shooterMotorID2);
    
    public Shooter(){
    }

    public void setSpeed(double speed){
        shooter1.set(speed);
        shooter2.set(-speed);
    }
    public void incrementRPM(double joystickInput){
        desiredVelocity = desiredVelocity + (5*-joystickInput);
        pid.setSetpoint(desiredVelocity);
    }
    public void setRPM(double RPM){
        desiredVelocity = RPM;
        pid.setSetpoint(desiredVelocity);
    }

    public double getRPM(){
       
        return shooter1.getVelocity().getValueAsDouble()*60;
    }
    /**Meters per second **/
    public double estimatedFuelVelocity(){
        return (getRPM()*Constants.MathConstants.shooterWheelCircumference)/2;
    }
   
    public void periodic(){
        double output = pid.calculate(getRPM());
        setSpeed(output);
        SmartDashboard.putNumber("ShooterRPM", getRPM());
        SmartDashboard.putNumber("Fuel Velocity", estimatedFuelVelocity());
    }
}
