package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.tan;

public class DistanceSensorLocalization {
    HashMap<DistanceSensor, Pose2d> dsDict;
    public DistanceSensorLocalization(HashMap<DistanceSensor, Pose2d> distanceSensors,
                                      double topOffset, double bottomOffset, double leftOffset, double rightOffset){
        dsDict = distanceSensors;
        WallType.setOffsets(topOffset, bottomOffset, leftOffset, rightOffset); //should probably be hardcoded in
    }
    double getDistance(DistanceSensor ds, int trialCount){ //Kalman filter here?
        double dist = ds.getDistance(DistanceUnit.INCH);
        if (dist>80.0){
            return -1.0;
        }
        return dist;
//        return Math.abs(theoreticalDistance(new Pose2d(test.pose.vec().plus(
//                dsDict.get(ds).vec().rotated(test.pose.getHeading())),
//                dsDict.get(ds).getHeading()+test.pose.getHeading())));
    }
    double theoreticalDistance(Pose2d dsPose){
        WallType wall = getWallFromDSPose(dsPose);
        double perpendicular = wall.getPerpendicular(dsPose);
        double angleDiff = getAngleDiff(wall.heading, dsPose.getHeading());
        return (perpendicular/Math.cos(angleDiff));
    }
    public Pose2d refinePosition(Pose2d estimatedPose){
        StateField stateField = new StateField();
        HashMap<Vector2d, WallType> hitPoints = new HashMap<>();
        HashMap<DistanceSensor, Double> distances = new HashMap<>();
        for(DistanceSensor ds: dsDict.keySet()){
            double testDistance = getDistance(ds, 5);
            if(testDistance!=-1.0){ //TODO: check for validity using a different indicator than -1, redo
                distances.put(ds, testDistance);
            }
        }
        for(DistanceSensor ds : distances.keySet()){
            Pose2d dsRelativePose = dsDict.get(ds);
            Pose2d dsAbsolutePose = dsRelativePose.plus(estimatedPose);
            Vector2d hitpoint = Vector2d.polar(distances.get(ds), dsRelativePose.getHeading()).plus(dsRelativePose.vec());
            System.out.println(ds.getDeviceName());
            System.out.println(hitpoint);
            //System.out.println(dsRelativePose);
            System.out.println(getWallFromDSPose(dsAbsolutePose));
            hitPoints.put(hitpoint, getWallFromDSPose(dsAbsolutePose));
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
            }else if (hitpointList.size()!=0){
                stateField.addStateZone(hitpointList.get(0), wall);
            }
        }
        Pose2d testPose = stateField.getPredictedPose(estimatedPose);
        return simplifyPoseHeading(testPose);//simplifyPoseHeading(testPose.plus(new Pose2d(0.0,0.0,
                //lowestHeading(estimatedPose.getHeading(), testPose.getHeading()))));
    }
    WallType getWallFromDSPose(Pose2d pose){
        for(WallType wall : WallType.values()){
            if(checkWallCollision(wall, pose)){
                //System.out.println(pose);
                return wall;
            }
        }
        return null;
    }
    boolean checkWallCollision(WallType wall, Pose2d pose){
        //System.out.println("CHECK COLLISION");
        //System.out.println(wall);
        double angle = modularAngle(pose.getHeading());
        Vector2d[] endPoints = wall.endPoints();
        //System.out.println((endPoints[0].minus(pose.vec())).angle());
        //System.out.println((endPoints[1].minus(pose.vec())).angle());
        return isBetween(angle, (endPoints[0].minus(pose.vec())).angle(), (endPoints[1].minus(pose.vec())).angle());
        //TODO: might have some overlap issues, leverage the fact that the endpoints are in counter-clockwise order
    }
    boolean isBetween(double angle, double a, double b){
        //Two cases, cross over or not
//        if (b>a){ //no crossover
//            return (angle>=modularAngle(a) && angle<=modularAngle(b));
//        }else { //crossover
//            return (angle>=modularAngle(a) || angle<=modularAngle(b));
//        }
        double diff = getAngleDiff(a, b);
        return (getAngleDiff(angle, a)<=diff && getAngleDiff(angle, b)<=diff);
    }
    static double getAngleDiff(double a, double b){
        if (a<b){
            return getAngleDiff(b, a);
        }
        double diff=(a-b)%(2*PI);
        if(diff>PI){
            return 2*PI-diff;
        }
        return diff;
    }
    Pose2d simplifyPoseHeading(Pose2d pose){
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading()%(2*PI));
    }
    double lowestHeading(double estimatedPoseHeading, double testPoseHeading){
        double lowestDiff = Double.MAX_VALUE;
        int iValue = -1;
        for(int i =0; i<8; i++){
            if (modularAngle(getAngleDiff(testPoseHeading+i*PI/2, estimatedPoseHeading))<lowestDiff){
                iValue = i;
            }
        }
        return testPoseHeading+iValue*PI/4;
    }
    public static double modularAngle(double angle){
        if (angle<0.0){
            return 2*PI+angle%(2*PI);
        }
        return angle%(2*PI);
    }
}