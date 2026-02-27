package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TagTracking extends SubsystemBase{
    private AprilTagFieldLayout aprilTagFieldLayout = Constants.FieldSetpoints.aprilTagFieldLayout;
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
 private List<PhotonPipelineResult> results;
 PhotonCameraSim cameraSim;// = new PhotonCameraSim(camera);
    VisionSystemSim visionSim; //= new VisionSystemSim("Camera Sim");
    public TagTracking(String cameraName, Transform3d cameraTransform){
        camera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform);
        cameraSim = new PhotonCameraSim(camera);
        visionSim = new VisionSystemSim("Camera Sim");
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(aprilTagFieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, cameraTransform);

            cameraSim.enableDrawWireframe(true);
        }
      }
    public void periodic(){
        results = camera.getAllUnreadResults();
    }
    public List<PhotonPipelineResult> getAllUnreadResults() {
    return results;
  }
  

  public PhotonPipelineResult getLatestResult() {
    if(results.size()>0){
      return results.get(results.size()-1); 
    }
    return new PhotonPipelineResult();
  }
  public PhotonTrackedTarget getBestTarget() {
    return getLatestResult().getBestTarget();
  }

  public Boolean hasTargets() {
    if(!getAllUnreadResults().isEmpty()){
      PhotonPipelineResult target = getAllUnreadResults().get(getAllUnreadResults().size()-1);

      return target.hasTargets();
    } else {
      return false;
    }
    
  }

  public double getYaw() {
    return getBestTarget().getYaw();
  }
  public double getSkew() {
    return getBestTarget().getSkew();
  }
  public double getPitch() {
    return getBestTarget().getPitch();
  }
  public double getArea() {
    return getBestTarget().getArea();
  }

  public Transform3d getRobotTransform() {
    return getBestTarget().getBestCameraToTarget();
  }
public Boolean tagOnScreen(){
  if(hasTargets()&&getBestTarget()!= null){
    return true;
  }else{
    return false;
  }
}
  public double getApriltagDistance(Pose2d robotPose, int apriltagID) {
    double tagDist = 100000;
    if(hasTargets() && getBestTarget() != null) {

      tagDist = PhotonUtils.getDistanceToPose(robotPose, Constants.FieldSetpoints.aprilTagFieldLayout.getTagPose(apriltagID).get().toPose2d());
      
    }

    return tagDist;

  }

  public double getPoseAmbiguity() {
    
      return getBestTarget().getPoseAmbiguity();
    
    
  }

  public int getApriltagID() {
    return getBestTarget().getFiducialId();
  }

  public void getTagPose() {
    aprilTagFieldLayout.getTagPose(getApriltagID());
  }
  public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
   public void simPeriodic(Pose2d simPose) {
        visionSim.update(simPose);
    }
  public Optional<EstimatedRobotPose> getVisionBasedPose() {
    return photonPoseEstimator.update(getLatestResult());
  }
}
