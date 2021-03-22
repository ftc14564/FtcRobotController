package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous (name = "Autonomous2021")
public class Autonomous2021 extends LinearOpMode {
    final double qFieldLength = 72;
    final double powershotGap = 7.5;
    Vector2d stackLocation = new Vector2d(-qFieldLength/3, -qFieldLength/2);
    DcMotorEx intake;
    DcMotorEx grabber;
    DcMotorEx launch;
    MecanumDrivetrain drive;
    PositionState state = PositionState.Start;
    int ringState = 2;
    boolean pickedRings;
    boolean pickedGoal;
    boolean goalDeposited;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(new Pose2d(-qFieldLength * (2.0/3.0), -qFieldLength/3, 0.0));

        waitForStart();

        if (isStopRequested()) return;

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        grabber = hardwareMap.get(DcMotorEx.class, "wobbleGrab");
        launch = hardwareMap.get(DcMotorEx.class , "Launch");

        //grabber.setPower(-0.75);
        /*
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(0.0, 0.0), Math.toRadians(45.0))
                .splineTo(new Vector2d(qFieldLength/2, qFieldLength/2), Math.toRadians(45.0))
                .build();
        drive.followTrajectory(traj);
        */
        /*
        DetectStack(drive.getPoseEstimate());
        CollectStack(drive.getPoseEstimate());
        DepositWobbleGoal(drive.getPoseEstimate());
        ShootPowershot(drive.getPoseEstimate());
        CollectWobbleGoal(drive.getPoseEstimate());
        DepositWobbleGoal(drive.getPoseEstimate());

         */
        Park(drive.getPoseEstimate());

        drive.setMotorPowers(0,0,0,0);
    }
    void DetectStack(Pose2d initialPose){
        if (!pickedGoal) {
            if (initialPose.getX()<=stackLocation.getX()){
                if (initialPose.getY()<=-qFieldLength/3){
                    drive.followTrajectory(drive.trajectoryBuilder(initialPose)
                            .lineTo(new Vector2d(-qFieldLength*(2./3), -qFieldLength*(2./3)))
                            .build());
                }
                else{
                    drive.followTrajectory(drive.trajectoryBuilder(initialPose)
                            .lineTo(new Vector2d(-qFieldLength/6, 0))
                            .build());
                }
            }
        }
        Pose2d startPose = drive.getPoseEstimate();
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(getStackMeasurementPose(startPose, 15), setSpeed(15),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(traj);
        state = PositionState.Outside_Stack;
    }
    void CollectStack(Pose2d initialPose){
        TrajectoryBuilder traj = drive.trajectoryBuilder(initialPose);
        if (state != PositionState.Outside_Stack) {
            traj.lineToLinearHeading(getStackMeasurementPose(initialPose, 13));
            drive.followTrajectory(traj.build());
        }
        drive.turn(Math.toRadians(-5));
        //intake.setVelocity(-1200);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(16)
                .build());
    }
    void ShootRings(Pose2d initialPose){
        if (pickedRings){
            GoToCenter(initialPose);
        } else {
            GoToCenterCarefully(initialPose);
        }
        TrajectoryBuilder traj = drive.trajectoryBuilder(drive.getPoseEstimate());
        traj.lineToConstantHeading(new Vector2d(0, -qFieldLength/2.5));
        drive.followTrajectory(traj.build());
        state = PositionState.Ring_Goal;
        /*
        LaunchRing(9);
        for (int i = 0; i<3; i++){ // One extra for safety
            sleep(1000);
            LaunchRing(9);
        }

         */
    }
    void ShootPowershot(Pose2d initialPose){
        if (pickedRings){
            GoToCenter(initialPose);
        } else {
            GoToCenterCarefully(initialPose);
        }
        TrajectoryBuilder traj = drive.trajectoryBuilder(drive.getPoseEstimate());
        traj.lineToConstantHeading(new Vector2d(0, -qFieldLength/6));
        state = PositionState.Powershot;
        //LaunchRing(7);
        for (int i = 0; i<2; i++){
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(powershotGap)
                    .build());
           // LaunchRing(7);
        }
    }
    void GoToCenter(Pose2d initialPose){
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d())
                .build());
    }
    void GoToCenterCarefully(Pose2d initialPose){
        if (initialPose.getX()<stackLocation.getX()) {
            if (initialPose.getY() >= stackLocation.getY()) {
                drive.followTrajectory(drive.trajectoryBuilder(initialPose)
                        .lineTo(new Vector2d(-qFieldLength / 3, 0))
                        .build());
            } else {
                drive.followTrajectory(drive.trajectoryBuilder(initialPose)
                        .lineTo(new Vector2d(-qFieldLength / 6, -qFieldLength * (5/6)))
                        .build());
            }
        }
        GoToCenter(drive.getPoseEstimate());
    }
    void Park(Pose2d initialPose){
        if(initialPose.getX()>=-qFieldLength/6){ //lower part of wobble goal deposit
            if (initialPose.getY()<=-qFieldLength/3){
                drive.followTrajectory(drive.trajectoryBuilder(initialPose)
                        .lineToLinearHeading(new Pose2d(initialPose.getX(), 0,0))
                        .build()
                );
            }
        }
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(qFieldLength / 6, drive.getPoseEstimate().getY()))
                .build()
        );
    }
    void DepositWobbleGoal(Pose2d initialPose){
        TrajectoryBuilder trajCenter = drive.trajectoryBuilder(initialPose);
        TrajectoryBuilder trajBox;
        boolean trajBoxEmpty = false; //Assumed false unless known to be true
        if (!goalDeposited){
            if (initialPose.getY()>=stackLocation.getY() || initialPose.getX()>=stackLocation.getX()){
                trajCenter.lineToLinearHeading(new Pose2d(0, -qFieldLength/3, Math.toRadians(-90)));
                trajBox = drive.trajectoryBuilder(new Pose2d(0, -qFieldLength/6, Math.toRadians(-90)));
            }
            else {
                trajCenter.splineToSplineHeading(new Pose2d(-qFieldLength * (2./3), -qFieldLength * (2./3), 0),Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(0, -60, Math.toRadians(-90)),0);
                trajBox = drive.trajectoryBuilder(new Pose2d(0, -60, Math.toRadians(-90)));
                if (ringState == 0) trajBoxEmpty = true;
            }
        }
        else {
            if (initialPose.getX()<=stackLocation.getX()){
                trajCenter.splineToConstantHeading(new Vector2d(-qFieldLength/2, -qFieldLength/6), Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(0, -qFieldLength/6), Math.toRadians(-90));
            }
            else {
                trajCenter.lineToLinearHeading(new Pose2d(0, -qFieldLength / 6, Math.toRadians(-90)));
            }
            trajBox = drive.trajectoryBuilder(new Pose2d(0, -qFieldLength/6, Math.toRadians(-90)));
        }
        switch (ringState){
            case 0:
                trajBox.lineToConstantHeading(new Vector2d(0, -60));
                break;
            case 1:
                trajBox.lineToConstantHeading(new Vector2d(15, -40));
                break;
            case 2:
                trajBox.lineToConstantHeading(new Vector2d(45,-60));
                break;
        }
        drive.followTrajectory(trajCenter.build());
        if (!trajBoxEmpty) drive.followTrajectory(trajBox.build());
        /*
        if (!goalDeposited){
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(5)
                    .build());
        }
        */
        //grabber.setPower(0.5);
        //sleep(750);
        //grabber.setPower(-0.2);
        state = PositionState.Goal_Box;
    }
    void CollectWobbleGoal(Pose2d initialPose){
        Trajectory trajCenter = drive.trajectoryBuilder(initialPose)
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(-180)))
                .build();
        Trajectory trajGoal = drive.trajectoryBuilder(trajCenter.end())
                .splineTo(new Vector2d(-qFieldLength* (5./8), 0 ), Math.toRadians(-180))
                .build();
        drive.followTrajectory(trajCenter);
        //grabber.setPower(0.2);
        drive.followTrajectory(trajGoal);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(8)
                .build());
        //grabber.setPower(-0.75);
    }
    MinVelocityConstraint setSpeed(double speed){
        return new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(speed, DriveConstants.TRACK_WIDTH)
                )
        );
    }
    Pose2d getStackMeasurementPose(Pose2d currentPose, double radius){
        double d_x =  currentPose.getX()-stackLocation.getX();
        double d_y =  currentPose.getY()-stackLocation.getY();
        double scale = radius /
                Math.hypot(d_x, d_y);
        double angle = Math.atan(d_y/d_x);
        if (d_x>0) angle += Math.PI;
        return new Pose2d(
                stackLocation.getX() + d_x * scale,
                stackLocation.getY() + d_y * scale,
                angle);
    }
    void LaunchRing(float speed) {
        //TODO: FILL THIS IN
    }
    enum PositionState {
        Field, //Arbitrary position
        Start,
        Powershot,
        Ring_Goal,
        Goal_Box,
        Outside_Stack
    }
}