package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings;

public interface GamepadTriggerMap extends GamepadMap{

    GamepadTriggerMap left = GamepadTriggerMap.gamepad.left;
    GamepadTriggerMap right = GamepadTriggerMap.gamepad.right;
    enum gamepad implements GamepadTriggerMap {
        left,
        right;
    }
}
