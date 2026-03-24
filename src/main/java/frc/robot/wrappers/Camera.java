// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;
import java.io.IOException;
import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Statics;
import frc.robot.data.VisionEstimationStrategy;
import frc.robot.Constants;
import frc.robot.FeatureFlags;

/** Add your docs here. */
public class Camera extends PhotonCamera {
    private List<PhotonPipelineResult> m_cameraData;
    private boolean m_cameraDataRecent = false;
    private Transform3d m_cameraOffset;
    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d m_previousTransform = Transform3d.kZero;

    private PhotonPoseEstimator m_poseEstimator;

    public boolean disablePoseEstimation = false;

    public Camera(String name, Transform3d cameraOffset) {
        super(name);
        m_cameraOffset = cameraOffset;
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        if (FeatureFlags.kVisionEstimationStrategy == VisionEstimationStrategy.kMultiTag) {
            m_poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, cameraOffset);
        }
    }
    public ArrayList<Integer> getVisibleAprilTagIds() {
        updateVisible();
        List<PhotonTrackedTarget> targets = this.getVisibleAprilTags();
        ArrayList<Integer> out = new ArrayList<Integer>();
        for (Integer i = 0; i < targets.size(); i++) {
            out.add(targets.get(i).fiducialId);
        }
        return out;
    }

    public List<PhotonTrackedTarget> getVisibleAprilTags() {
        updateVisible();
        return m_cameraData.get(0).getTargets();
    }

    public Optional<PhotonTrackedTarget> getBestAprilTag() {
        if (!populateCameraData()) return Optional.empty();
        try {
            if (m_cameraData.size() == 0 || m_cameraData.get(0) == null) {
                return Optional.empty();
            }
            if (m_cameraData.get(0).hasTargets()) {
                List<PhotonTrackedTarget> output = m_cameraData.get(0).getTargets();
                double bestAngle = 0;
                PhotonTrackedTarget bestTarget = null;
                for (int i = 0; i < output.size(); i++) {
                    PhotonTrackedTarget target = output.get(i);
                    Transform3d transform = target.bestCameraToTarget;
                    double angle = transform.getRotation().getZ();
                    if (Math.abs(angle) > bestAngle) {
                        bestTarget = target;
                        bestAngle = angle;
                    }
                }
                if (bestTarget != null) {
                    return Optional.of(bestTarget);
                }
            }
        } catch (Throwable err) {
            System.err.println("An Exception happened whilst trying to get the best sensed april tag: ");
            System.err.println(err);
        }
        return Optional.empty();
    }

    
    public Optional<PhotonTrackedTarget> getMostRecentBestAprilTag() {
        updateVisible();
        return m_bestAprilTag;
    }

    public PhotonPipelineResult getTargets() {
        updateVisible();
        return m_cameraData.get(0);
    }

    public Optional<EstimatedRobotPose> estimatePose() {
        populateCameraData();
        if (!updateVisible()) {
            return Optional.empty();
        }
        if (disablePoseEstimation) {
            return Optional.empty();
        }
        if (m_cameraData.size() == 0 || m_cameraData.get(0) == null) {
            return Optional.empty();
        }
        if (!m_cameraData.get(0).hasTargets()) {
            return Optional.empty();
        }
        PhotonPipelineResult result = m_cameraData.get(0);

        Optional<EstimatedRobotPose> estimatedPoseOptional = Optional.empty();
        if (FeatureFlags.kVisionEstimationStrategy == VisionEstimationStrategy.kMultiTag) {
            estimatedPoseOptional = m_poseEstimator.estimateCoprocMultiTagPose(result);
        }
        if (estimatedPoseOptional.isEmpty()) {
            estimatedPoseOptional = m_poseEstimator.estimateLowestAmbiguityPose(result);
        }
        return estimatedPoseOptional;
    }

    public Optional<Pose2d> estimatePose2d(Pose2d robotPose) {
        populateCameraData();
        if (updateVisible()) {
            if (disablePoseEstimation) {
                return Optional.empty();
            }
            if (m_cameraData.size() == 0 || m_cameraData.get(0) == null) {
                return Optional.empty();
            }
            if (!m_cameraData.get(0).hasTargets()) {
                return Optional.empty();
            }
            if (FeatureFlags.kVisionEstimationStrategy == VisionEstimationStrategy.kMultiTag) {
                /**
                 * Uses a PhotonPoseEstimator
                 */
                PhotonPipelineResult result = m_cameraData.get(0);
                Optional<EstimatedRobotPose> estimatedPoseOptional = m_poseEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPoseOptional.isEmpty()) {
                    estimatedPoseOptional = m_poseEstimator.estimateLowestAmbiguityPose(result);
                }
                if (estimatedPoseOptional.isPresent()) {
                    EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
                    
                    double ambiguity = 0;

                    if (estimatedPose.strategy == PoseStrategy.LOWEST_AMBIGUITY) {
                        double totalAmbiguity = 0;
                        int ambiguityCount = 0;
                        for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                            totalAmbiguity += target.poseAmbiguity;
                            ambiguityCount++;
                        }
                        ambiguity = ((totalAmbiguity / ambiguityCount) * 10) + 0.2;
                    } else {
                        // Trust because it is extremly reliable
                        ambiguity = 0.5;
                    }

                    return Optional.of(estimatedPose.estimatedPose.toPose2d().interpolate(robotPose, Math.min(ambiguity, 1)));
                }
            } else {
                PhotonTrackedTarget target = m_cameraData.get(0).getBestTarget();
                if (target == null) {
                    return Optional.empty();
                }
                
                Transform3d targetTransform3d = target.bestCameraToTarget;
                // System.out.print("Input ");
                // System.out.print(targetTransform3d);
                if (target.poseAmbiguity > Constants.kMaxPoseAmbiguity) {
                    return Optional.empty();
                }
                if (target.bestCameraToTarget.getX() > Constants.kMaxTagDistance) {
                    return Optional.empty();
                }
                int target_id = target.fiducialId;
                Pose3d field_target_pose = aprilTagFieldLayout.getTagPose(target_id).get();
                Pose3d camera_pose = field_target_pose.plus(targetTransform3d.inverse());
                Pose3d calculatedRobotPose = camera_pose.plus(m_cameraOffset.inverse());
                if (FeatureFlags.kVisionEstimationStrategy == VisionEstimationStrategy.kSingleTagFiltered) {
                    return Optional.of(calculatedRobotPose.toPose2d());
                } else {
                    return Optional.of(calculatedRobotPose.toPose2d().interpolate(robotPose, Math.min(target.poseAmbiguity*10 + 0.2,1)));
                }
            }
        } else {
            return Optional.empty();
        }
        return Optional.empty();
    }

    private Optional<PhotonTrackedTarget> m_bestAprilTag = Optional.empty();
    private boolean updateVisible() {
        final boolean result = populateCameraData();
        final Optional<PhotonTrackedTarget> best = this.getBestAprilTag();
        if (best.isPresent()) {
            m_bestAprilTag = best;
        }
        return result;
    }

    private boolean populateCameraData() {
        List<PhotonPipelineResult> results = this.getAllUnreadResults();
        if (results.size() == 0) {
            return false;
        }
        m_cameraData = results;
        m_cameraDataRecent = true;
        return true;
    }
}