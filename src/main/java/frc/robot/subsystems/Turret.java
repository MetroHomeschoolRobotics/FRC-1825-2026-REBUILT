package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    private static CANcoder angle = new CANcoder(Constants.MotorIDs.turretCANID);
        private TalonFX turret = new TalonFX(Constants.MotorIDs.turretMotorID);
        
        private static double robotAngle;
        private static double rotationCount = 0;
        private static int rotationCountInt = 0;
    
        
        private static PIDController pid = new PIDController(0.001, 0, 0);
        public Turret(){
    
        }
        public static void turretSetSetpoint(double setpoint){
            if(setpoint>180){
                setpoint-=360;
            }else if(setpoint<-180){
                setpoint+=360;
            }
            pid.setSetpoint(setpoint);
            
            
        }
        public static void setRobotAngle(double angle){
            robotAngle = angle;
        }
        /**use the cancoder position with the 10:1 gear ratio to get the actual angle,
         * it also adds the robot angle
         * 
         * 
         */
        public double getGearedAngle(){
            //360/10 cuz gear ratios 
            double output = angle.getAbsolutePosition().getValueAsDouble()*36;
            // if(output>18){
            //     output-=36;
            // }else if(output<-18){
            //     output+=36;
            // }
            output+=rotationCountInt*36;
            return output+robotAngle;

        }
    //     public static double getAbsoluteAngle(){
    //         double output =angle.getAbsolutePosition().getValueAsDouble()*360;
           
    //         if(output>180){
    //             output-=360;
    //             // rotationCount+=0.5;
    //         }else if(output<-180){
                
    //             output+=360;
    //             // rotationCount-=0.5;
    //         }
    //         return output;
    // }

    public void periodic(){
        //DOES THIS WORK, IDK
       rotationCount=angle.getPosition().getValueAsDouble();
       rotationCountInt= (int) rotationCount;
       
        double output = pid.calculate(getGearedAngle());
        turret.set(output);

    }


}
