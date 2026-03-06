package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    //TO/DO fix the magic numbers here
    private Spark hood1 = new Spark(1);
    
    private CANdi CANDi= new CANdi(0); 
    
    private Spark hood2 = new Spark(2);
    private PIDController pid = new PIDController(Constants.PIDConstants.hoodP, Constants.PIDConstants.hoodI, Constants.PIDConstants.hoodD);

    private PWMSim hood1Sim = new PWMSim(Constants.MotorIDs.hoodID1);
    private CANdiSimState CANdiSim = new CANdiSimState(CANDi);

    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier simNotifier = null;
    private double lastSimTime = 0.0;
    public Hood(){
        hood1.addFollower(hood2);
        
    }
    public void setSpeed(double speed){
        hood1.set(speed);
       
    }
    public void setPID(double setPoint){
        pid.setSetpoint(setPoint);
        
    }
    public double getAngle(){
        return Constants.MathConstants.defaultHoodAngle-(CANDi.getPWM1Position().getValueAsDouble()*Constants.MathConstants.hoodRotationsPerDegree)
        ;
    }
    

    public void periodic(){
        double output = pid.calculate(getAngle());
        setSpeed(output);
        hood1Sim.setSpeed(output);
        
    }
    public void simulationPeriodic(){
        //This dont work
        
        double output = pid.calculate(getAngle());
        CANdiSim.setPwm1Velocity(output);
        setSpeed(output);
        
        double speed = hood1Sim.getSpeed();
        
        SmartDashboard.putNumber("simulated hood position", speed);
        SmartDashboard.putNumber("hood angle", getAngle());
    }
}
