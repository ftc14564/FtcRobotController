package org.firstinspires.ftc.teamcode.drive.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.*;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadButtonMap;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadStickMap;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadTriggerMap;


@TeleOp
public class Teleop2021 extends LinearOpMode {
    void CreateGamepadActions(){
        new GamepadActionBuilder(gamepad1)
        .Add(GamepadButtonMap.a, () -> {return;})
        .Add(GamepadButtonMap.b, () -> {return;})
        .Add(GamepadStickMap.left, (x, y) -> {return;});
    }
    public void runOpMode(){
        while (!isStopRequested()) {
            GamepadAction.RunActions();
        }
    };
}
