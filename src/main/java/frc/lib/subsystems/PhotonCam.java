package frc.lib.subsystems;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

@Logged
public class PhotonCam extends SubsystemBase {
  private PhotonCamera cam;
  private StructPublisher<Pose3d> publisher;
  private StructArrayPublisher<Pose3d> trackedPub;
  private Transform3d offset;

  // The field from AprilTagFields will be different depending on the game.
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Construct PhotonPoseEstimator
  private PhotonPoseEstimator photonPoseEstimator;

  // Loggers hate him! Watch Matrix crash Epilogue with this one simple trick!
  @NotLogged
  private Matrix<N3, N1> confidenceMatrix = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 100, 100, 10000 }));

  private Alert cameraOffline;

  public PhotonCam(String camera, Transform3d robotToCam) {
    offset = robotToCam;
    cam = new PhotonCamera(camera);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        robotToCam);

    cameraOffline = new Alert("Camera "+camera+" is offline!", AlertType.kError);

    publisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/vision/"+camera+"/pose", Pose3d.struct).publish();
    trackedPub = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/vision/"+camera+"/targets", Pose3d.struct).publish();
  }

  public void periodic() {
    // Alert if camera is offline
    cameraOffline.set(!cam.isConnected());

    // Process results detected since last run
    for (PhotonPipelineResult result : cam.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> o = photonPoseEstimator.estimateCoprocMultiTagPose(result);

      if (o.isPresent()) {
        EstimatedRobotPose e = o.get();

        publisher.set(e.estimatedPose);

        trackedPub.set(
          e.targetsUsed.stream().<Pose3d>map((tt) -> e.estimatedPose.plus(offset.plus(tt.bestCameraToTarget))).toArray(Pose3d[]::new)
        );
        

        // Display on map
        RobotContainer.getRobot().swerve.field.getObject(cam.getName()).setPose(e.estimatedPose.toPose2d());
        RobotContainer.getRobot().swerve.field.getObject(cam.getName()+"_targets").setPoses(
          e.targetsUsed.stream().<Pose2d>map((tt) -> e.estimatedPose.plus(offset.plus(tt.bestCameraToTarget)).toPose2d()).toList()
        );

        // Calculate our current equation, just for dashbard display to compare
        double area = 0;
        double ambiguity = 0;
        double score = 0;
        for (PhotonTrackedTarget x : e.targetsUsed) {
          double amb = x.getPoseAmbiguity();

          area += x.getArea();
          ambiguity += x.getPoseAmbiguity();

          // Don't use overly ambiguous poses or invalid (-1) poses
          if (amb < 0) {
            // Tags estimated as a MultiTag are invalid, but are extremely accurate
            // Hell, double it!
            score += x.getArea() * 2;
          }
          // Docs suggest anything above 0.2 is useless
          // Scale down score down to nothing at or beyond 0.2
          else if (amb < 0.2) {
            score += x.getArea() * (x.getPoseAmbiguity() * -5 + 1);
          }
        }

        
        double calculatedConf = Math.max(score * (-9.5 / 1.5) + 10, 0.5);

        SmartDashboard.putNumber("vision/"+cam.getName() + "/total area", area);
        SmartDashboard.putNumber("vision/"+cam.getName() + "/ambiguity", ambiguity);
        SmartDashboard.putNumber("vision/"+cam.getName() + "/score", score);

        SmartDashboard.putNumber("vision/"+cam.getName() + "/calculated conf", calculatedConf);

        double xy = calculatedConf;
        double r = calculatedConf * 5;
       
        // Fill existing matrix instead of making a new one to save on memory
        // allocations
        confidenceMatrix.set(0, 0, xy);
        confidenceMatrix.set(1, 0, xy);
        confidenceMatrix.set(2, 0, r);
        if (inField(e)) {
          RobotContainer.getRobot().swerve.swerveOdometry.addVisionMeasurement(
              e.estimatedPose.toPose2d(),
              e.timestampSeconds,
              confidenceMatrix);
        }
      }
    }
  }


 

  // TODO should this have an offset inwards/outwards?

  public boolean inField(EstimatedRobotPose e) {
    {
      if (e.estimatedPose.getY() > FIELD_WIDTH) {
        return false;
      } else if (e.estimatedPose.getY() < 0) {
        return false;
      }
      if (e.estimatedPose.getX() > FIELD_LENGTH) {
        return false;
      } else if (e.estimatedPose.getX() < 0) {
        return false;
      }
      if (e.estimatedPose.getZ() > MAX_VISION_ESTIMATE_Z) {
        return false;
      } else if (e.estimatedPose.getZ() < MIN_VISION_ESTIMATE_Z) {
        return false;
      }
      return true;
    }
  }
}
