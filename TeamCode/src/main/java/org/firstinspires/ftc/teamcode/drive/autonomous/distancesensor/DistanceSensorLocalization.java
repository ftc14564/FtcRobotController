package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Vector;

import static java.lang.Math.PI;
import static java.lang.Math.sin;
import static java.lang.Math.tan;

public class DistanceSensorLocalization {
    HashMap<DistanceSensor, Pose2d> dsDict;
    DistanceSensorLocalization(HashMap<DistanceSensor, Pose2d> distanceSensors,
    double topOffset, double bottomOffset, double leftOffset, double rightOffset){
        dsDict = distanceSensors;
        WallType.setOffsets(topOffset, bottomOffset, leftOffset, rightOffset);
    }
    double getDistance(DistanceSensor ds, int trialCount){
        //Kalman filter here???
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
    void refinePosition(Pose2d estimatedPose){
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
        for (Vector2d hitpoint: hitPoints.keySet()){

        }
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
