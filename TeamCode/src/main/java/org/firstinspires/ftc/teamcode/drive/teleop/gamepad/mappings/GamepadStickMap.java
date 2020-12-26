package org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings;

import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadMap;

public interface GamepadStickMap extends GamepadMap {
    byte type = 3;
    GamepadStickMap left = gamepad.left;
    GamepadStickMap right = gamepad.right;
    enum gamepad implements GamepadStickMap {
    left,
    right;
    }
}
