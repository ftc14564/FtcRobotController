package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Math.PI;

public enum WallType{
    TOP(0.0),
    LEFT(PI/2),
    BOTTOM(PI),
    RIGHT(3*PI/2);

    double heading;
    double positiveOffset;
    double negativeOffset;
    WallType(double v) {
        heading = v;
    }
    boolean inBounds(double position){
        return position<=positiveOffset && position>=negativeOffset;
    }
    double getPerpendicular(Pose2d pose){
        switch(ordinal()){
            case 0:
                return positiveOffset-pose.getX();
            case 2:
                return negativeOffset-pose.getX();
            case 1:
                return positiveOffset-pose.getY();
            case 3:
                return negativeOffset-pose.getY();
            default:
                return 0.0;
        }
    }
    Pose2d offsetPosition(double offset){
        switch(this){
            case TOP:
                return new Pose2d(72.0-offset, 0.0);
            case LEFT:
                return new Pose2d(0.0, 72.0-offset);
            case BOTTOM:
                return new Pose2d(-72.0+offset, 0.0);
            case RIGHT:
                return new Pose2d(0.0, -72.0+offset);
        }
        return new Pose2d();
    }
    WallType reverse(){
        switch(this){
            case TOP:
                return BOTTOM;
            case BOTTOM:
                return TOP;
            case LEFT:
                return RIGHT;
            case RIGHT:
                return LEFT;
        }
        return null;
    }
    static void setOffsets(double topOffset, double bottomOffset, double leftOffset, double rightOffset){
        TOP.positiveOffset = leftOffset; BOTTOM.positiveOffset = leftOffset;
        TOP.negativeOffset = rightOffset; BOTTOM.negativeOffset = rightOffset;
        LEFT.positiveOffset = topOffset; RIGHT.positiveOffset = topOffset;
        LEFT.negativeOffset = bottomOffset; LEFT.negativeOffset = bottomOffset;
    }
}