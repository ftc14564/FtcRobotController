package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RingDetect extends Autonomous2021{
    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        int c = -1;
        while (opModeIsActive()){
                c = getRingState();
                drive.forceSendTelemetryToDashboard("RingState", (double) c);
                sleep(250);
        }
        drive.setMotorPowers(0,0,0,0);
    }
}
