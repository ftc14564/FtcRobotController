package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;

import static java.lang.Math.tan;

public class DistanceSensorLocalization {
    HashMap<DistanceSensor, Pose2d> dsDict;
    public DistanceSensorLocalization(HashMap<DistanceSensor, Pose2d> distanceSensors,
                                      double topOffset, double bottomOffset, double leftOffset, double rightOffset){
        dsDict = distanceSensors;
        WallType.setOffsets(topOffset, bottomOffset, leftOffset, rightOffset); //should probably be hardcoded in
    }
    double getDistance(DistanceSensor ds, int trialCount){ //Kalman filter here?
        double distance = -1.0;
        for(int i = 0; i<trialCount; i++){
            double measuredDistance = ds.getDistance(DistanceUnit.INCH);
            if(measuredDistance>80.0) { //greater than measureable amnt (tune)
                distance*=2;
            }else {
                distance += measuredDistance;
            }
        }
        distance*=1.0/trialCount;
        return distance;
    }
    public ArrayList<Double> getDistances(){
        ArrayList<Double> list = new ArrayList<>();
        for (DistanceSensor ds: dsDict.keySet()){
            list.add(getDistance(ds, 5));
        }
        return list;
    }
    public Pose2d refinePosition(Pose2d estimatedPose){
        if(true) {
            return new Pose2d();
        }
        StateField stateField = new StateField();
        HashMap<Vector2d, WallType> hitPoints = new HashMap<>();
        HashMap<DistanceSensor, Double> distances = new HashMap<>();
        for(DistanceSensor ds: dsDict.keySet()){
            double testDistance = getDistance(ds, 5);
            if(testDistance!=-1.0){
                distances.put(ds, testDistance);
            }
        }
        for(DistanceSensor ds : distances.keySet()){
            Pose2d dsAbsolutePose = dsDict.get(ds).plus(estimatedPose);
            hitPoints.put(Vector2d.polar(distances.get(ds), dsAbsolutePose.getHeading()).plus(estimatedPose.vec()),
                    getWallFromDSPose(dsDict.get(ds)));
        }
        for (WallType wall : WallType.values()){
            ArrayList<Vector2d> hitpointList = new ArrayList<>();
            for (Vector2d hitpoint: hitPoints.keySet()){
                if (hitPoints.get(hitpoint)==wall){
                    hitpointList.add(hitpoint);
                }
            }
            if (hitpointList.size()>=2){
                stateField.addStateLine(hitpointList.get(0), hitpointList.get(1), wall);
                //TODO: make a more accurate implementation for 3+ measurements
            }else{
                stateField.addStateZone(hitpointList.get(0), wall);
            }
        }
        return stateField.getPredictedPose(estimatedPose);
    }
    WallType getWallFromDSPose(Pose2d pose){
        for(WallType wall : WallType.values()){
            if(checkWallCollision(wall, pose)){
                return wall;
            }
        }
        return null;
    }

    boolean checkWallCollision(WallType wall, Pose2d pose){
       double theta = pose.getHeading() - wall.heading;
       double collisionOffset = tan(theta)*wall.getPerpendicular(pose);
       return wall.inBounds(collisionOffset);
    }

}
