package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;


public class Shooter extends SubsystemBase{
    private PIDController pid = new PIDController(0.01,0,0);
    
    private double desiredVelocity = 0;

    private SparkMax shooter = new SparkMax(Constants.MotorIDs.shooterMotorID, SparkLowLevel.MotorType.kBrushless);
    
    public Shooter(){
    }

    public void setSpeed(double speed){
        shooter.set(speed);
    }
    public void incrementRPM(double joystickInput){
        desiredVelocity = desiredVelocity + (10*-joystickInput);
        pid.setSetpoint(desiredVelocity);
    }
    public void setRPM(double RPM){
        desiredVelocity = RPM;
        pid.setSetpoint(desiredVelocity);
    }

    public double getVelocity(){
       
        return shooter.getEncoder().getVelocity();
    }
    public void periodic(){
        double output = pid.calculate(getVelocity());
        shooter.set(output);
    }
}
