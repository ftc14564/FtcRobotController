package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.HashMap;

import kotlin.Pair;

public class StateField {
    HashMap<WallType, Pair<Double, Double>> stateLines;
    HashMap<WallType, Pair<Double, Double>> stateZones;
    StateField(double errorMargin){

    }
    void addStateZone(Vector2d hitpoint, WallType wall){

    }
    void addStateLine(Vector2d hitpoint1, Vector2d hitpoint2, WallType wall){

    }
    Pose2d getStateLinePosition(WallType wall){
        Pose2d averagePose = wall.offsetPosition(stateLines.get(wall).getFirst());
        averagePose = averagePose.plus(new Pose2d(0.0, 0.0, stateLines.get(wall).getSecond()));
        //TODO: might have to cross-reference with the current pose for the real heading
        // (theoretically both normal and reversed should be the same)
        if(stateLines.keySet().contains(wall.reverse())){
            averagePose = (averagePose).plus(wall.reverse().offsetPosition(stateLines.get(wall.reverse()).getFirst())).div(2.0);
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
        Pose2d linearPosAngle = getStateLinePosition((WallType) stateLines.keySet().toArray()[0]);
        if (stateLines.size()!=0){
            return mergePoses( getStateLinePosition((WallType) stateLines.keySet().toArray()[0]), linearPosAngle);
        } else {
            if (stateZones.size()==0){
                return mergePoses(currentPose, linearPosAngle);
            }else{
                return mergePoses(currentPose, linearPosAngle); //TODO: Replace with zone positioning
            }
        }
    }
    Pose2d mergePoses(Pose2d currentPose, Pose2d desiredPose){
        if(desiredPose.getX() == 0.0){
            return new Pose2d(currentPose.getX(), desiredPose.getY(), desiredPose.getHeading());
        } else{
            return new Pose2d(desiredPose.getX(), currentPose.getY(), desiredPose.getHeading());
        }
    }
}
