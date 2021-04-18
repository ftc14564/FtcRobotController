package org.firstinspires.ftc.teamcode.drive.autonomous.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.HashMap;

import static java.lang.Math.PI;

@Autonomous
public class test extends LinearOpMode {
    public static Pose2d pose = new Pose2d(0.0, 15.0,  PI/2);
    @Override
    public void runOpMode(){
        waitForStart();

        HashMap<DistanceSensor, Pose2d> distanceSensors = new HashMap<>();

        distanceSensors.put(hardwareMap.get(DistanceSensor.class, "DS_rightFront"), new Pose2d(3.0,-7.0,-PI/2));
        distanceSensors.put(hardwareMap.get(DistanceSensor.class, "DS_rightRear"), new Pose2d(-3.0,-7.0,-PI/2));
        distanceSensors.put(hardwareMap.get(DistanceSensor.class, "DS_leftFront"), new Pose2d(3.0,7.0, PI/2));
        distanceSensors.put(hardwareMap.get(DistanceSensor.class, "DS_leftRear"), new Pose2d(-3.0,7.0, PI/2));

        DistanceSensorLocalization dsl = new DistanceSensorLocalization(distanceSensors, 72.0, -72.0, 72.0, -72.0);
        //System.out.println(dsl.getWallFromDSPose(new Pose2d(60.0, 60.0, PI/5)));
        System.out.println(dsl.refinePosition(pose)); //TODO: FIX FOR WHEN DS IS THOUGHT TO BE OUTSIDE FIELD

    }
}
