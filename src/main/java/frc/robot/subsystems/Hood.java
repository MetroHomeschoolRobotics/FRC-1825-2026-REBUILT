package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

// import com.ctre.phoenix6.hardware.CANdi;
// import com.ctre.phoenix6.sim.CANdiSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.Notifier;

public class Hood extends SubsystemBase {
    
    
    
    //private CANdi CANDi= new CANdi(Constants.MotorIDs.CANDiId); 
    private Servo hood1 = new Servo(Constants.MotorIDs.hoodID1);
    private Servo hood2 = new Servo(Constants.MotorIDs.hoodID2);
    private PIDController pid = new PIDController(Constants.PIDConstants.hoodP, Constants.PIDConstants.hoodI, Constants.PIDConstants.hoodD);

    private PWMSim hood1Sim = new PWMSim(Constants.MotorIDs.hoodID1);
    private double CorrectedSetpoint;
   // private CANdiSimState CANdiSim = new CANdiSimState(CANDi);

    // private static final double kSimLoopPeriod = 0.002; // 2 ms
    // private Notifier simNotifier = null;
    // private double lastSimTime = 0.0;
    public Hood(){
        //hood1.addFollower(hood2);
        
    }
    public void setSpeed(double speed){
        //hood1.set(speed);
       
    }

    public void setSetpoint(double setPoint){
        setPoint = setPoint + 0.2;
        if (setPoint >= 0.33) {
            setPoint = 0.33;
        }
        // Based on how the servo class works with the actuonix
        // fully in to fully out on the actuanix relates to 2/9 to 7/9 in the servo class
        // So 0.22 on the servo corelates with fully in on the actuonix
        hood1.set(setPoint);
        hood2.set(setPoint);
    }
    public void setSetpointJoystick(double setPoint) {
        CorrectedSetpoint = (setPoint+1)/6;
        // setPoint comes in as a value from -1 to 1
        // Convert that into a value from 0 to 0.3333
        setSetpoint(CorrectedSetpoint);
    }
    public void setSetpointDirect(double setPoint) {
        // Use this one if the value comes in from 0 to 0.33
        setSetpoint(setPoint);
    }
    public double getAngle(){
       // return Constants.Setpoints.defaultHoodAngle-(CANDi.getPWM1Position().getValueAsDouble()*Constants.MathConstants.hoodRotationsPerDegree)
        return hood1.get();
    }
    public double getPwmPosition(){
       // return CANDi.getPWM1Position().getValueAsDouble();
       return 0;
    }

    public void periodic(){
        //double output = pid.calculate(getAngle());
        //setSpeed(output);
        //hood1Sim.setSpeed(output);
        SmartDashboard.putNumber("hood angle", getAngle());
        
    }
    public void simulationPeriodic(){
        //This dont work
        
        double output = pid.calculate(getAngle());
        //CANdiSim.setPwm1Velocity(output);
        setSpeed(output);
        
        double speed = hood1Sim.getSpeed();
        
        SmartDashboard.putNumber("simulated hood position", speed);
        SmartDashboard.putNumber("hood angle", getAngle());
    }
}
