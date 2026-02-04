package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

public final class Constants {
    public static class MotorIDs{
        public static final int shooterMotorID1 = 14;
        public static final int shooterMotorID2 = 15;
        public static final int intakeID = 16;
        public static final int indexerID = 17;
        public static final int beltID = 18;
        public static final int turretMotorID = 19;
        public static final int turretCANID=20;
        public static final int hoodID1=21;
        public static final int hoodID2=22;
    }
    public static class MathConstants{
        public static final double hoodRotationsPerDegree=1.0;
        public static final double defaultHoodAngle = 58.337;
    }
    public static class FieldSetpoints{
        // uncomment when the april tag layout is released
        public static final Pose2d blueHubPose = new Pose2d();
        public static final Pose2d redHubPose = new Pose2d();
        //public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField();
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
}
