package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions;

public interface GamepadFunction {
    default byte returnType(){
        return 0;
    }
}
