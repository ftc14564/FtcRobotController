package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions;

public interface GamepadButtonFunction extends GamepadFunction {
    void run();
    @Override
    default byte returnType(){
        return 1;
    }
}
