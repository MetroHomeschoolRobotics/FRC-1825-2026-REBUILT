package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class MotorIDs{
        public static final int shooterMotorID1 = 14;
        public static final int shooterMotorID2 = 15;

        public static final int intakeID = 16;
        public static final int intakeRetractorID = 17;
        public static final int intakeCANcoderID = 26;

        public static final int indexerID = 18;
        public static final int indexerLeftID = 31;
        public static final int indexerRightID=32;
        public static final int beltID = 19;

        public static final int turretMotorID = 20;
        public static final int turretCANcoderID=22;

        public static final int hoodID1=0;
        public static final int hoodID2=1;//PWM channels

        public static final int CANDiId = 25;
    }
   
    public static class MathConstants{
        public static final double hoodRotationsPerDegree=1.0;

        public static final double shooterWheelRadius = Units.inchesToMeters(4.0/2.0); //diameter/2
        public static final double shooterWheelCircumference = 2*Math.PI*shooterWheelRadius;
    }
    public static class FieldSetpoints{
      
        public static final Pose2d blueHubPose = new Pose2d(Units.inchesToMeters(181.56),Units.inchesToMeters(158), Rotation2d.kZero);
        public static final Pose2d redHubPose = new Pose2d(Units.inchesToMeters(469.62),Units.inchesToMeters(158), Rotation2d.kZero);
        
         public static final double blueAllianceZoneX= Units.inchesToMeters(144.19);
        public static final double redAllianceZoneX = Units.inchesToMeters(464.63);
        public static final double upperYValue = Units.inchesToMeters(181.5+29);
        public static final double lowerYValue = Units.inchesToMeters(181.5-29);
        
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }
    public static class TimerConstants{
        //
        public static final double transitionStart = 140;
        public static final double shift1Start = 130;
        public static final double shift2Start = 105;
        public static final double shift3Start = 80;
        public static final double shift4Start = 55;
        public static final double endgameStart = 30;
        public static final double[] shiftTimes= {
            transitionStart,
            shift1Start,
            shift2Start,
            shift3Start,
            shift4Start,
            endgameStart
        };
    
 
    }
    public static class Setpoints{
        public static final double defaultHoodAngle = 58.337;
        public static final double passingHoodAngle = 45;
        public static final double retractorDeploySpeed = -0.3;
        public static final double retractorRetractSpeed = 0.75; //TODO driven
        public static final double indexerSpeed = 0.7;
        public static final double beltSpeed = 0.5;
        public static final double intakeSpeed = 0.4; //TODO driven
        public static final double turretForwardSoftLimit = 18;
        public static final double turretReverseSoftLimit = -19;
    }   
    public static class TimeOfFlightLUT{
        public static final double[] inputs = {
            1.7,//0
            2.6,//1
            3.6,//2
            4.3//3 
        };
         public static final double[] outputs = {
            1.235,//0
            1.347,//1 
            1.624,//2
            1.632//3
        };
    }
    public static class InterpolationData{
      
        public static final double[] inputs = {
            1.07,//0
            2.02,//1
            2.55,//2
            2.68,//3
            2.91,//4
            3.22,//5
            3.65,//6
            3.85,//7
            4.64,//8
            5.3, //9
            5.7 //10
        };
        private static double rpmdrop = 0;
        public static final double[] outputs = {
           2885,//0
           3224,//1
           3427,//2
           3537,//3
           3781,//4
           3873,//5
           4149,//6
           4222,//7
           4764,//8
           5132,//9
           5750,//10
           
        };
        
    }
    public static class PIDConstants{
        public static final double hoodP =0.01; // % output / degree
        public static final double hoodI =0; // % output / (degree * second)
        public static final double hoodD =0; // % output / (degree / second)

        public static final double shooterP = 0.000205; // % output / RPM error
        public static final double shooterI = 0.00;//12; // % output / (RPM * second)
        public static final double shooterD= 0.000000145;//14; // % output  / (RPM / second) ALSO (% output * second) / RPM 

        public static final double shooterKs = 0; // % Output (constant)
        public static final double shooterKv = 0.000166; // 1/6000, % Output / RPM
        public static final double shooterKa = 0; // % output / (RPM*second)
        
        public static final double turretP = 0.02; // % output / degree
        public static final double turretI = 0.000; // % output / (degree * second)
        public static final double turretD = 0; // % output / (degree / second)
    }
    public static class CameraPositions {
    public static final Transform3d frontLeftTranslation = new Transform3d(
        Units.inchesToMeters(-10.6),
        Units.inchesToMeters(10.8), // 11.29
        Units.inchesToMeters(6.09),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(-30),
            Units.degreesToRadians(90)));

//     public static final Transform3d frontRightTranslation = new Transform3d(
//         Units.inchesToMeters(-10.9),
//         Units.inchesToMeters(10.8), // -11.29
//         Units.inchesToMeters(9.2),
//         new Rotation3d(
//             Units.degreesToRadians(0),
//             Units.degreesToRadians(0),
//             Units.degreesToRadians(0)));
                                                              
   }
   public enum TurretMode { HUB, PASSING, NEUTRAL, HUBSOTM };
 }
