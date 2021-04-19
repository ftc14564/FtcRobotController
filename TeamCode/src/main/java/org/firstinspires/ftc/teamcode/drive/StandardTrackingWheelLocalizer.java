package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV_V = 8192;
    public static double TICKS_PER_REV_H = 8192;
    public static double WHEEL_RADIUS_V = 0.5; // in
    public static double WHEEL_RADIUS_H = 0.5; // in
    public static double GEAR_RATIO = 24.0/17.0; // output (wheel) speed / input (encoder) speed
    //normally gear ratio, but we will use for traction rate
    //this probably needs to be fixed mechanically
    public static double LATERAL_DISTANCE = 7.9; // in; distance between the left and right wheels (needs to be tuned probably) //used to be 8.25
    public static double FORWARD_OFFSET = -7; // in; offset of the lateral wheel (needs to be tuned probably)

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public Double[] ticks_inch = new Double[]{0.0, 0.0, 0.0};

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "WobbleGoal"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches_H(double ticks) {
        return WHEEL_RADIUS_H * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV_H;
    }
    public static double encoderTicksToInches_V(double ticks) {
        return WHEEL_RADIUS_V * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV_V;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        ticks_inch = new Double[]{
                encoderTicksToInches_V(leftEncoder.getCurrentPosition()),
                encoderTicksToInches_V(rightEncoder.getCurrentPosition()),
                encoderTicksToInches_H(frontEncoder.getCurrentPosition())
        };
        return Arrays.asList(ticks_inch);
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches_V(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches_V(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches_H(frontEncoder.getCorrectedVelocity())
        );
    }
}
