package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
        return inBounds(position, 0.0);
    }
    boolean inBounds(double position, double offset){
        return position<=positiveOffset+offset && position>=negativeOffset+offset;
    }
    double getPerpendicular(Pose2d pose){
        switch(this){
            case TOP:
                return LEFT.positiveOffset-pose.getX();
            case BOTTOM:
                return LEFT.negativeOffset-pose.getX();
            case LEFT:
                return TOP.positiveOffset-pose.getY();
            case RIGHT:
                return TOP.negativeOffset-pose.getY();
            default:
                return 0.0;
        }
    }
    Pose2d offsetPosition(double offset){
        switch(this){
            case TOP:
                return new Pose2d(LEFT.positiveOffset-offset, 0.0);
            case LEFT:
                return new Pose2d(0.0, TOP.positiveOffset-offset);
            case BOTTOM:
                return new Pose2d(LEFT.negativeOffset+offset, 0.0);
            case RIGHT:
                return new Pose2d(0.0, TOP.negativeOffset+offset);
        }
        return new Pose2d();
    }
    boolean inBounds(double position, Pose2d pose){
        switch (this){
            case TOP:
            case BOTTOM:
                return inBounds(position, -pose.getY());
            case LEFT:
            case RIGHT:
                return inBounds(position, -pose.getX()); //TODO: FIX IF THERE IS A PARITY ISSUE
        }
        return false;
    }
    WallType reverse(){
        switch(this){
            case TOP:
                return BOTTOM;
            case LEFT:
                return RIGHT;
            case BOTTOM:
                return TOP;
            case RIGHT:
                return LEFT;
        }
        return null;
    }
    Vector2d[] endPoints(){
        switch(this){
            case TOP:
                return new Vector2d[]{
                        new Vector2d(LEFT.positiveOffset, negativeOffset),
                        new Vector2d(LEFT.positiveOffset, positiveOffset)
                };
            case LEFT:
                return new Vector2d[]{
                        new Vector2d(positiveOffset, TOP.positiveOffset),
                        new Vector2d(negativeOffset, TOP.positiveOffset)
                };
            case BOTTOM:
                return new Vector2d[]{
                        new Vector2d(LEFT.negativeOffset, positiveOffset),
                        new Vector2d(LEFT.negativeOffset, negativeOffset)
                };
            case RIGHT:
                return new Vector2d[]{
                        new Vector2d(negativeOffset, TOP.negativeOffset),
                        new Vector2d(positiveOffset, TOP.negativeOffset)
                };
        }
        return new Vector2d[]{
            new Vector2d(), new Vector2d()
        };
    }
//    WallType getFromHeading(double heading){
//        if (heading==0.0){
//            return TOP;
//        }
//        if (heading==PI/2){
//            return LEFT;
//        }
//        if (heading==PI){
//            return BOTTOM;
//        }
//        if (heading==3*PI/2){
//            return RIGHT;
//        }
//        return null;
//    }
    static void setOffsets(double topOffset, double bottomOffset, double leftOffset, double rightOffset){
        TOP.positiveOffset = leftOffset; BOTTOM.positiveOffset = leftOffset;
        TOP.negativeOffset = rightOffset; BOTTOM.negativeOffset = rightOffset;
        LEFT.positiveOffset = topOffset; RIGHT.positiveOffset = topOffset;
        LEFT.negativeOffset = bottomOffset; RIGHT.negativeOffset = bottomOffset;
    }
}