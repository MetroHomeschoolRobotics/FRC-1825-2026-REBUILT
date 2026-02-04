package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    private static CANcoder angle = new CANcoder(Constants.MotorIDs.turretCANID);
        private TalonFX turret = new TalonFX(Constants.MotorIDs.turretMotorID);
        private Timer timer = new Timer();
        private static double robotAngle;
        
        private static PIDController pid = new PIDController(0.001, 0, 0);
        public Turret(){
    
        }
        public static void turretSetSetpoint(double setpoint){
            pid.setSetpoint(setpoint);
        }
        public static void setRobotAngle(double angle){
            robotAngle = angle;
        }
        public static double getAbsoluteAngle(){
            double output =angle.getAbsolutePosition().getValueAsDouble()*360+robotAngle;
            if(output>180){
                output-=360;
            }
            return output;
    }

    public void periodic(){
        double output = pid.calculate(getAbsoluteAngle());
        turret.set(output);
    }


}
