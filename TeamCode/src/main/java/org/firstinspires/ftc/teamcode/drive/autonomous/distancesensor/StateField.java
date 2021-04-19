package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.HashMap;

import kotlin.Pair;

import static org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor.DistanceSensorLocalization.modularAngle;

public class StateField {
    HashMap<WallType, Pair<Double, Double>> stateLines = new HashMap<>();
    HashMap<WallType, Pair<Double, Double>> stateZones = new HashMap<>();
    void reset(){
        stateLines.clear();
        stateZones.clear();
    }
    void addStateZone(Vector2d hitpoint, WallType wall){
        //literally does nothing since there's no reason to implement it yet //TODO: implement this
    }
    void addStateLine(Vector2d hitpoint1, Vector2d hitpoint2, WallType wall){
        if(hitpoint1.angle()<hitpoint2.angle()){
            addStateLine(hitpoint2, hitpoint1, wall);
            return;
        }
        System.out.print("Wall ");
        System.out.println(wall);
        System.out.print("H1 ");
        System.out.println(hitpoint1);
        System.out.print("H2 ");
        System.out.println(hitpoint2);
        Vector2d distance = hitpoint1.minus(hitpoint2);
        double angle = distance.angleBetween(hitpoint2);
        Pair line = new Pair<>(Math.sin(angle)*hitpoint2.norm(), absoluteAngle(wall.heading+Math.PI/2-distance.angle()));
        System.out.println("Heading ");
        System.out.println(absoluteAngle(-Math.toDegrees(wall.heading-Math.PI/2-distance.angle())));
        System.out.println("Offset ");
        System.out.println(Math.sin(angle)*hitpoint2.norm());
        stateLines.put(wall, line);
    }
    Pose2d getStateLinePosition(){
        WallType wall = (WallType) stateLines.keySet().toArray()[0];
        Pose2d averagePose = wall.offsetPosition(stateLines.get(wall).getFirst());
        averagePose = averagePose.plus(new Pose2d(0.0, 0.0, stateLines.get(wall).getSecond()));
        //TODO: might have to cross-reference with the current pose for the real heading
        // (theoretically both normal and reversed should be the same)
        if(stateLines.containsKey(wall.reverse())){
            averagePose = (averagePose).plus(wall.reverse().offsetPosition(stateLines.get(wall.reverse()).getFirst()))
                    .plus(new Pose2d(0.0, 0.0, stateLines.get(wall.reverse()).getSecond())).div(2.0);
            stateLines.remove(wall);
            stateLines.remove(wall.reverse());
        } else {
            stateLines.remove(wall);
        }
        return averagePose;
    }
    Pose2d getPredictedPose(Pose2d currentPose){
        if (stateLines.size()==0){
            return currentPose; //TODO: Replace with complex angular positioning (hitpoints alone?)
        }
        Pose2d stateLinePose = getStateLinePosition();
        if (stateLines.size()!=0){
            return mergePoses(getStateLinePosition(), stateLinePose);
        } else {
            if (stateZones.size()==0){
                return mergePoses(currentPose, stateLinePose);
            }else{
                return mergePoses(currentPose, stateLinePose); //TODO: Replace with zone positioning
            }
        }
    }
    Pose2d mergePoses(Pose2d currentPose, Pose2d desiredPose){
        //TODO: replace desired pose unknown symbol from 0.0 to another value to remove edge cases
        Pose2d finalPose = new Pose2d();
        if(desiredPose.getX() == 0.0){
            finalPose = finalPose.plus(new Pose2d(currentPose.getX()));
        } else {
            finalPose = finalPose.plus(new Pose2d(desiredPose.getX()));
        }
        if(desiredPose.getY() == 0.0){
            finalPose = finalPose.plus(new Pose2d(0.0, currentPose.getY()));
        } else {
            finalPose = finalPose.plus(new Pose2d(0.0, desiredPose.getY()));
        }
        finalPose = finalPose.plus(new Pose2d(0.0, 0.0, desiredPose.getHeading()));
        return finalPose;
    }
    double absoluteAngle(double angle){
        double clampedAngle = angle %(2*Math.PI);
        if(clampedAngle<-Math.PI){
            return clampedAngle+2*Math.PI;
        }
        if(clampedAngle>Math.PI){
            return clampedAngle-2*Math.PI;
        }
        return clampedAngle;
    }
}