package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private TalonFX intake = new TalonFX(Constants.MotorIDs.intakeID,"*");
    private TalonFX intakeRetractor = new TalonFX(Constants.MotorIDs.intakeRetractorID,"*");
    //private CANcoder angle = new CANcoder(Constants.MotorIDs.intakeCANcoderID);
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private double defaultCANcoderAngle = 0;//add this to the angle to make 0 intake down
    
    private Mechanism2d intake2d = new Mechanism2d(Units.inchesToMeters(20), Units.inchesToMeters(4));
   
    private final DCMotor TopIndexDCMotors = DCMotor.getKrakenX60Foc(1);
    private final LinearSystem<N2, N1, N2> TopIndexFlywheelSystem = LinearSystemId.createDCMotorSystem(TopIndexDCMotors, 0.0005, 1);
    private final DCMotorSim TopIndexFlywheelSim = new DCMotorSim(TopIndexFlywheelSystem, TopIndexDCMotors);

    public Intake(){
        setConfigs();
        intake.getConfigurator().apply(config);
    }
    public void setIntakeSpeed(double speed){
        intake.set(speed);
        
    }
    private void setConfigs(){
        config.CurrentLimits.StatorCurrentLimit = 40;
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
