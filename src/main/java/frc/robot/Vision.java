package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision{

    private static AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private static PhotonCamera cmrMavi = new PhotonCamera(Constants.VisionCameraKargaConstants.Name);
    private static PhotonCamera cmrYesil = new PhotonCamera(Constants.VisionCameraTavukConstants.Name);
    private static PhotonCamera cmrSiyah = new PhotonCamera(Constants.VisionCameraTavukConstants.Name);


    private static PhotonPoseEstimator estMavi = new PhotonPoseEstimator(FieldLayout, Constants.VisionCameraKargaConstants.RobotToCamera);
    private static PhotonPoseEstimator estYesil = new PhotonPoseEstimator(FieldLayout, Constants.VisionCameraTavukConstants.RobotToCamera);
    private static PhotonPoseEstimator estSiyah = new PhotonPoseEstimator(FieldLayout, Constants.VisionCameraSiyahConstants.RobotToCamera);

            
    static PhotonCamera[] cameras = {cmrMavi, cmrYesil, cmrSiyah};
    static PhotonPoseEstimator[] poseEstimators = {estMavi, estYesil, estSiyah};
            
    private static Matrix<N3, N1> CurrentSTDDevs;
                    
    public Vision(){
                
    }
                
    public static void getVisionEstimates(int cameraNum){
        PhotonCamera camera = cameras[cameraNum];
        PhotonPoseEstimator poseEstimator = poseEstimators[cameraNum];
        Optional<EstimatedRobotPose> visEst = Optional.empty();
    
        for(PhotonPipelineResult result : camera.getAllUnreadResults()){
            visEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (visEst.isEmpty()){
                visEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            setVisionSTDDevs(visEst, result.getTargets(), poseEstimator);
            visEst.ifPresent(est -> {
            Constants.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), result.getTimestampSeconds(), getCurrentStdDevs());
        });
    }
}
                                                
    private static void setVisionSTDDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEstimator){
        boolean deletePose = false;
        if (estimatedPose.isEmpty()){
            CurrentSTDDevs = Constants.VisionStdConstants.stdSingleTag;
        } else {
            Matrix<N3, N1> estStdDevs = Constants.VisionStdConstants.stdSingleTag;
            int targetNum = 0;
            double avgDist = 0;                                
                                
            for (PhotonTrackedTarget trg : targets){
                if (trg.poseAmbiguity > Constants.VisionStdConstants.MaxAmbiguity) deletePose = true;
                Optional<Pose3d> TagPose = poseEstimator.getFieldTags().getTagPose(trg.fiducialId);
                                
                if (TagPose.isEmpty()) continue;
                targetNum ++;
                avgDist += TagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }
                                
            if (targetNum == 0){
                CurrentSTDDevs = Constants.VisionStdConstants.stdSingleTag;
            } else {
                avgDist /= targetNum;
                                
                if (targetNum > 1) CurrentSTDDevs = Constants.VisionStdConstants.stdMultitag;
                if (targetNum == 1 && avgDist > Constants.VisionStdConstants.AvgMaxDistance)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * (avgDist / Constants.VisionStdConstants.StdPenalty)));
                                
                if (!deletePose) CurrentSTDDevs = estStdDevs;
                else CurrentSTDDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
        }
    }                
                                
    public static Matrix <N3, N1> getCurrentStdDevs(){
        return CurrentSTDDevs;
    }

    public static int getCameraCount(){
        return cameras.length;
    }
}
