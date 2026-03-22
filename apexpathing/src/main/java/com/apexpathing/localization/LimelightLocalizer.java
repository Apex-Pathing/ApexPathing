package com.apexpathing.localization;

import com.apexpathing.geometry.Pose2d;
import com.apexpathing.util.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * Limelight Class localizer for apriltags
 * @author Xander Haemel - 31616 404 not found
 */
public class LimelightLocalizer  {
    Limelight3A limelight3A;
    LLResult limelightResult;
    public void setPipeline(int pipelineNumber){
        limelight3A.pipelineSwitch(pipelineNumber);
    }
    public LimelightLocalizer(HardwareMap hardwareMap, @NotNull String LimelightName){
        limelight3A = hardwareMap.get(Limelight3A.class, LimelightName);
    }

    /**
     * get fidicual results
     * @return the List of apriltags with their respective ID's
     */
    public List<LLResultTypes.FiducialResult> getTagIDs(){
        return limelightResult.getFiducialResults();
    }

    /**
     * updates the limelight Result
     */
    public void update() {
        limelightResult = limelight3A.getLatestResult();
    }

    /**
     * Gets a pose 2d from the limelight assuming it's setup correctly
     * @return the limelight in Pose 2D,
     */
    public Pose getPose() {
        Pose3D botPose = limelightResult.getBotpose_MT2();
        double x = botPose.getPosition().x;
        double y = botPose.getPosition().y;
        double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
        Pose returnPose = new Pose(x, y, heading);
        return returnPose;
    }


}
