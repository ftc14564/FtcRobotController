package org.firstinspires.ftc.teamcode.drive.teleop.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions.GamepadButtonFunction;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions.GamepadFunction;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions.GamepadStickFunction;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.functions.GamepadTriggerFunction;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadButtonMap;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadMap;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadStickMap;
import org.firstinspires.ftc.teamcode.drive.teleop.gamepad.mappings.GamepadTriggerMap;

import java.util.ArrayList;

public class GamepadActionBuilder {
    static ArrayList<Gamepad> gamepads;
    static Gamepad currentGamepad;
    public GamepadActionBuilder(Gamepad gamepad){
        if (!gamepads.contains(gamepad)){
            gamepads.add(gamepad);
        }
        currentGamepad = gamepad;
    }
    GamepadActionBuilder Build(GamepadMap map, GamepadFunction function){
        GamepadAction.AddAction(new GamepadAction(currentGamepad, map, function));
        return this;
    }
    GamepadActionBuilder Build(GamepadMap map1, GamepadMap map2, GamepadFunction function){
        GamepadAction.AddAction(new GamepadAction(currentGamepad, map1, map2, function));
        return this;
    }
    public GamepadActionBuilder Add(GamepadButtonMap map, GamepadButtonFunction function){
        return Build(map, function);
    }
    public GamepadActionBuilder Add(GamepadTriggerMap map, GamepadTriggerFunction function){
        return Build(map, function);
    }
    public GamepadActionBuilder Add(GamepadStickMap map, GamepadStickFunction function){
        return Build(map, function);
    }
    public GamepadActionBuilder Add(GamepadMap map1, GamepadMap map2, GamepadButtonFunction function){
        return Build(map1, map2, function);
    }
    public GamepadActionBuilder Add(GamepadTriggerMap map1, GamepadButtonMap map2, GamepadTriggerFunction function){
        return Build(map1, map2, function);
    }
    public GamepadActionBuilder Add(GamepadStickMap map1, GamepadButtonMap map2, GamepadStickFunction function){
        return Build(map1, map2, function);
    }
}


