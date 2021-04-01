package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions;

public interface GamepadTriggerFunction extends GamepadFunction {
    void run(float n);
    @Override
    default byte returnType(){
        return 2;
    }
}
