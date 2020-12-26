package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings;

import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadMap;

public interface GamepadButtonMap extends GamepadMap {
        GamepadButtonMap a = gamepad.a;
        GamepadButtonMap b = gamepad.b;
        GamepadButtonMap x = gamepad.x;
        GamepadButtonMap y = gamepad.y;
        GamepadButtonMap start = gamepad.start;
        GamepadButtonMap back = gamepad.back;

        enum gamepad implements GamepadButtonMap{
                a,
                b,
                x,
                y,
                start,
                back;
        }
        enum dpad implements GamepadButtonMap {
                up,
                down,
                left,
                right;
        }
        enum bumper implements GamepadButtonMap {
                left,
                right;
        }
        enum stick implements GamepadButtonMap {
                left,
                right;
        }
}