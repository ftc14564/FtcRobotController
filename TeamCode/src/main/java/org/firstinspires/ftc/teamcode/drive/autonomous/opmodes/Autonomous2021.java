package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.opencv.OpenCVHelper;
import org.firstinspires.ftc.teamcode.opencv.RingDetector;
import org.firstinspires.ftc.teamcode.opencv.UltimateGoalCVHelper;
import org.opencv.core.Mat;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous (name = "Autonomous2021")
public class Autonomous2021 extends LinearOpMode {
    //Configurables
    public static int visionEnabled = 1; //Stand in for boolean
    public static double shooterPower = -0.9;

    OpenCVHelper helper;
    UltimateGoalCVHelper vision;
    DcMotor in_front;
    DcMotor in_back;
    Servo wobbleGrab;

    DcMotor shooter;
    MecanumDrivetrain drive;
    int state = 2;
    final boolean RedAlliance = false;
    int FP;//Field Parity
    final double sq = 24.0;
    Pose2d startPose;
    void initializeRobot(){
        if (visionEnabled == 1){
            helper  = new OpenCVHelper();
            helper.initializeOpenCVAndVuforiaCamera(hardwareMap, "Internal" , FRONT , false);
            vision = new UltimateGoalCVHelper();
        }

        if (RedAlliance) {
            FP = -1;
        } else {
            FP = 1;
        }
        drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startPose = new Pose2d(-5*sq/2 , FP*sq, 0.0);
        drive.setPoseEstimate(startPose);

        in_front = hardwareMap.dcMotor.get("in_front");
        in_back = hardwareMap.dcMotor.get("in_back");
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
        shooter = hardwareMap.dcMotor.get("Launch");


        waitForStart();

        pickUpWobbleGoal();

        if (isStopRequested()) return;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        drive.setMotorPowers(0,0,0,0);
    }
    ArrayList<Trajectory> shootPowershot(){

        ArrayList<Trajectory> list = new ArrayList<>();
        TrajectoryBuilder builder1 = drive.trajectoryBuilder(startPose);
        builder1
                .lineToLinearHeading(PoseP(-1.4*sq, 0.4*sq, -188.3));
        list.add(builder1.build());

        TrajectoryBuilder builder2 = drive.trajectoryBuilder(list.get(list.size()-1).end());
            builder2.lineToLinearHeading(PoseP(-sq *(3.0/4), sq/2, -180.0));

         list.add(builder2.build());

        return list;
    }
    ArrayList<Trajectory> oneWobbleGoalAndShoot(Vector2d endPos){
        ArrayList<Trajectory> list = new ArrayList<Trajectory>();

        TrajectoryBuilder builder2 = drive.trajectoryBuilder(new Pose2d(endPos, Math.toRadians(-90.0)))
                .lineTo(VectorP(-sq/2, 3*sq/2));

        list.add(builder2.build());

        /* run between detectStack and builder1
            drive.turn(FP*atan(1.0/2.0)+(90.0).toRadians)
            state = getRingState()
            drive.turn(-(90.0).toRadians)
        */

        /* run between builder1 and builder2
        if (RedAlliance == (state == 1)){
            drive.turn((180.0).toRadians)
        }
        dropOffWobbleGoal()
         */
        /* run between builder2 and builder3
        drive.turn((-90).toRadians)

        shootRings()
         */
        endPos =  list.get(list.size()-1).end().vec();

        TrajectoryBuilder builder3 = drive.trajectoryBuilder(new Pose2d(endPos, Math.toRadians(180.0)))
                .back(sq);

        list.add(builder3.build());

        return list;
    }
    ArrayList<Trajectory> doRingStackAndWobbleGoalPreload() {
        ArrayList<Trajectory> list = new ArrayList<Trajectory>();

        TrajectoryBuilder builder1 = drive.trajectoryBuilder(PoseP(new Vector2d(-3*sq/2, startPose.getY()), 0.0), FP*Math.toRadians(0.0));
        builder1
                //.addDisplacementMarker( () -> {turnOnIntake();})
                .splineTo(VectorP(-sq, sq*(3.0/4)), 0.0);
               // .splineTo(VectorP(-sq/2, sq), FP*Math.toRadians(0.0));

        switch (state) {
            case 0: {
                builder1
                        .splineTo(VectorP(0.0,  5 * sq/2 ), FP * Math.toRadians(90.0));
            }
            break;
            case 1: {
                builder1
                        .splineTo(VectorP(sq,  3 * sq/2), FP*Math.toRadians(-90.0));
            }
            break;
            case 2: {
                builder1
                        .splineTo(VectorP(2*sq, 5* sq/2), FP*Math.toRadians(90.0));
            }
            break;
        }
        /*
        builder1
                .addDisplacementMarker( () ->{ turnOffIntake() ;});
                */

        list.add(builder1.build());
        return list;
        /* run between detectStack and builder1
            drive.turn(FP*atan(1.0/2.0)+(90.0).toRadians)
            state = getRingState()
            drive.turn(-drive.getPoseEstimate.getHeading())
        */
    }
    ArrayList<Trajectory> doRingStackAndWobbleGoal() {
        ArrayList<Trajectory> list = new ArrayList<Trajectory>();

        TrajectoryBuilder builder1 = drive.trajectoryBuilder(PoseP(new Vector2d(-3*sq/2, startPose.getY()), 45.0), FP*Math.toRadians(45.0));
        builder1
                .addDisplacementMarker( () -> {turnOnIntake();})
                .splineTo(VectorP(-sq, 3*sq/2), FP*Math.toRadians(45.0));

        switch (state) {
            case 0: {
                builder1
                        .splineTo(VectorP(0.0,  5 * sq/2 ), FP * Math.toRadians(90.0));
            }
            break;
            case 1: {
                builder1
                        .splineTo(VectorP(sq,  3 * sq/2), FP*Math.toRadians(-90.0));
            }
            break;
            case 2: {
                builder1
                        .splineTo(VectorP(2*sq, 5* sq/2), FP*Math.toRadians(90.0));
            }
            break;
        }
        builder1
                .addDisplacementMarker( () ->{ turnOffIntake() ;});
        list.add(builder1.build());
        return list;
    }
    ArrayList<Trajectory> detectStack()
    {
        ArrayList<Trajectory> list = new ArrayList<Trajectory>();
        TrajectoryBuilder builder1 = drive.trajectoryBuilder(startPose);
        builder1
                .lineTo(new Vector2d(-3*sq/2, startPose.getY()));
        list.add(builder1.build());
        return list;
    }
    ArrayList<Trajectory> doWobbleGoalAfterPickup(Pose2d endPose) {
        ArrayList<Trajectory> list = new ArrayList<>();
        TrajectoryBuilder builder1 = drive.trajectoryBuilder(endPose);
        switch (state) {
            case 0: {
                builder1
                        .lineTo(VectorP(0.0, 5 * sq / 2));
            }
            break;
            case 1: {
                builder1
                        .lineTo(VectorP(sq, 3 * sq / 2));
            }
            break;
            case 2: {
                builder1
                        .lineTo(VectorP(2 * sq, 5 * sq / 2));
            }
            break;
        }
        list.add(builder1.build());
        return list;
    }
    ArrayList<Trajectory> pickupWobbleGoal(Pose2d endPose){
        ArrayList<Trajectory> list = new ArrayList<>();
        TrajectoryBuilder builder1 = drive.trajectoryBuilder(endPose);
            builder1.lineTo(endPose.vec().minus(new Vector2d(10.0, 0)));
        list.add(builder1.build());
        //turn 180 deg
        TrajectoryBuilder builder2 = drive.trajectoryBuilder(
                new Pose2d(list.get(list.size()-1).end().vec(), Math.toRadians(90.0)));
            builder2.lineTo(new Vector2d(-1.5*sq, 2.25*sq));
        list.add(builder2.build());
        return list;
    }
    ArrayList<Trajectory> oneWobbleGoal(Vector2d endPos){
        ArrayList<Trajectory> list = new ArrayList<Trajectory>();

        TrajectoryBuilder builder2;

        switch (state) {
            case 0: {
                builder2 = drive.trajectoryBuilder(new Pose2d(endPos, Math.toRadians(-90.0)), FP*Math.toRadians(-90.0))
                        .splineTo(VectorP(0.0, 2*sq), FP*Math.toRadians(-90.0))
                        .splineToConstantHeading(VectorP(sq/2, 3*sq/2), 0.0);
            }
            break;
            case 1: {
                builder2 = drive.trajectoryBuilder(new Pose2d(endPos, Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(sq/2, endPos.getY()));
            }
            break;
            case 2:{
                builder2 = drive.trajectoryBuilder(new Pose2d(endPos, Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(sq/2, endPos.getY()));
            }
            break;
            default:
                throw new IllegalStateException("Unexpected value: " + state);
        }
        list.add(builder2.build());
        /* run between detectStack and builder1
        drive.turn(FP*atan(1.0/2.0)+(90.0).toRadians)
        state = getRingState()
        drive.turn(-(90.0).toRadians)
         */

        /* run between builder1 and builder2
        if (RedAlliance == (state == 1)){
            drive.turn((180.0).toRadians)
        }
        dropOffWobbleGoal()
         */

        return list;
    }
    Vector2d VectorP(double x, double y) {
        return new Vector2d(x, FP * y);
    }
    Pose2d PoseP(double x, double y, double headingDeg ) {
        return new Pose2d(x, FP * y, FP * Math.toRadians(headingDeg));
    }
    Pose2d PoseP(Vector2d VectorP, double headingDeg) {
        return new Pose2d(VectorP, FP * Math.toRadians(headingDeg));
    }
    void dropOffWobbleGoal() {
        wobbleGrab.setPosition(1);
    }
    void pickUpWobbleGoal(){
        wobbleGrab.setPosition(0);
    }
    int getRingState(){
        if (visionEnabled == 1){
            //implemented at lab (OpenCV code)
            char c = '?';
            for (int i = 0; i<5; i++){
                if (c=='?'){
                    c = vision.detectRings(helper);
                    sleep(100);
                }
            }
            switch (c){
                case 'A':
                    drive.sendTelemetryToDashboard("RingState", 0.0);
                    return 0;
                case 'B':
                    drive.sendTelemetryToDashboard("RingState", 1.0);
                    return 1;
                case 'C':
                    drive.sendTelemetryToDashboard("RingState", 2.0);
                    return 2;
            }
        }
        drive.sendTelemetryToDashboard("RingState", -1.0);
        return 0;
    }
    void turnOnIntake(){
        in_front.setPower(-1);
        in_back.setPower(-1);
    }
    void turnOffIntake(){
        in_front.setPower(0);
        in_back.setPower(0);
    }
    void shootRings(){
        in_front.setPower(-1);
        shooter.setPower(shooterPower);
        sleep(3000);
        in_front.setPower(0);
        shooter.setPower(0);
    }
}