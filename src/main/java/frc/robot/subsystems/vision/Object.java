package frc.robot.subsystems.vision;


import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Object extends SubsystemBase {
    private PhotonCamera camera;
 private List<PhotonPipelineResult> results;
 private Transform3d cameraTransform;
    public Object(String _cameraName, Transform3d _cameraTransform){
        camera = new PhotonCamera(_cameraName);
        cameraTransform = _cameraTransform;
        //photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform)
    }
    public void periodic(){
        results = camera.getAllUnreadResults();
    }
    public List<PhotonPipelineResult> getAllUnreadResults() {
    return results;
  }
  

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult(); // TODO remove the depricated function
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

  public Transform3d getObjectTransform() {
    
    return getBestTarget().getBestCameraToTarget().plus(cameraTransform) ;
  }
  public Pose2d getObjectPose(Pose2d robotPose){
    Transform3d transform= getObjectTransform();
    double fuelX= transform.getX()+robotPose.getX();
    double fuelY= transform.getY()+robotPose.getY();
    Pose2d objectPose = new Pose2d(fuelX,fuelY, robotPose.getRotation());
    return objectPose;
  }
  public boolean checkDistance(Pose2d robotPose){
    
    Pose2d objectPose = getObjectPose(robotPose); 
    double dist = Math.sqrt(Math.pow((objectPose.getX()-robotPose.getX()), 2)+Math.pow((objectPose.getY()-robotPose.getY()), 2));
    return dist<2;
  }
public Boolean objectOnScreen(){
  if(hasTargets()&&getBestTarget()!= null){
    return true;
  }else{
    return false;
  }
}


 


}
