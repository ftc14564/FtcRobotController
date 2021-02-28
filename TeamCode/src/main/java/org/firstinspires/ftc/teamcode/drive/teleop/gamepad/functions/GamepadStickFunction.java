package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions;

public interface GamepadStickFunction extends GamepadFunction {
    void run(float x, float y);
    @Override
    default byte returnType(){
        return 3;
    }
}
