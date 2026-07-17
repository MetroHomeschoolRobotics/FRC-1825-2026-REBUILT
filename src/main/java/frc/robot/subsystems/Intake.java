package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private TalonFX intake = new TalonFX(Constants.MotorIDs.intakeID,"*");
    private TalonFX intakeRetractor = new TalonFX(Constants.MotorIDs.intakeRetractorID,"*");
    //private CANcoder angle = new CANcoder(Constants.MotorIDs.intakeCANcoderID);
    private TalonFXConfiguration config = new TalonFXConfiguration();

    // private double defaultCANcoderAngle = 0;//add this to the angle to make 0 intake down

    public Intake(){
        setConfigs();
        intake.getConfigurator().apply(config);
    }
    public void setIntakeSpeed(double speed){
        intake.set(speed);
        
    }
    private void setConfigs(){
        config.CurrentLimits.StatorCurrentLimit = 80;
        //config.MotorOutput.Inverted =InvertedValue.CounterClockwise_Positive;
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
    public double getIntakeAngle(){
        return 0;//(angle.getAbsolutePosition().getValueAsDouble()*360)+defaultCANcoderAngle;
    }
    public void periodic(){
        SmartDashboard.putNumber("intakeAngle", getIntakeAngle());
    }
    public void simulationPeriodic(){
        
    }
}
