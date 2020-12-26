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
import java.util.HashMap;
import java.util.Map;

public class GamepadAction {
    static Map<GamepadMap, Boolean> gamepadBoolMap = new HashMap<GamepadMap, Boolean>();
    static Map<GamepadMap, Float> gamepadFloatMap = new HashMap<GamepadMap, Float>();
    static Map<GamepadMap, FloatTuple> gamepadFloatsMap = new HashMap<GamepadMap, FloatTuple>();

    static ArrayList<GamepadAction> actions;
    public Gamepad gamepad;
    public GamepadMap gamepadMap;
    public GamepadMap gamepadMap2 = null;
    public GamepadFunction gamepadFunction;
    public GamepadAction(Gamepad gamepad_, GamepadMap map, GamepadFunction function){
        gamepad = gamepad_;
        gamepadMap = map;
        gamepadFunction = function;
    }
    public GamepadAction(Gamepad gamepad_, GamepadMap map, GamepadMap map2, GamepadFunction function){
        gamepad = gamepad_;
        gamepadMap = map;
        gamepadMap2 = map2;
        gamepadFunction = function;
    }
    public static void AddAction(GamepadAction action){
        actions.add(action);
    }
    public static void RunActions(){
        for(GamepadAction action: actions){
            action.RunAction(action.gamepadFunction);
        }
    }
    void RunAction(GamepadFunction function){
        if (getMapping(gamepadMap) && getMapping(gamepadMap2)) {
            switch (function.returnType()) {
                case 1:
                    ((GamepadButtonFunction) function).run();
                    break;
                case 2:
                    ((GamepadTriggerFunction) function).run(gamepadFloatMap.get(gamepadMap)); //the first mapping is the one that should be used
                    break;
                case 3:
                    FloatTuple floatTuple = gamepadFloatsMap.get(gamepadMap);
                    ((GamepadStickFunction) function).run(floatTuple.x, floatTuple.y); //the first mapping is the one that should be used
                    break;
            }
        }
    }
    boolean getMapping(GamepadMap map){
        if (gamepadBoolMap.containsKey(map)){
            return gamepadBoolMap.get(map);
        }
        if (gamepadFloatMap.containsKey(map)){
            return true; //these will be floats, so true
        }
        if (gamepadFloatsMap.containsKey(map)){
            return true; //these will be floats, so true
        }
        return false;
    }
    void updateMappings() {
        gamepadBoolMap.put(null, true);
        gamepadBoolMap.put(GamepadButtonMap.a, gamepad.a);
        gamepadBoolMap.put(GamepadButtonMap.b, gamepad.b);
        gamepadBoolMap.put(GamepadButtonMap.x, gamepad.x);
        gamepadBoolMap.put(GamepadButtonMap.y, gamepad.y);
        gamepadBoolMap.put(GamepadButtonMap.start, gamepad.back);
        gamepadBoolMap.put(GamepadButtonMap.dpad.up, gamepad.dpad_up);
        gamepadBoolMap.put(GamepadButtonMap.dpad.down, gamepad.dpad_down);
        gamepadBoolMap.put(GamepadButtonMap.dpad.left, gamepad.dpad_left);
        gamepadBoolMap.put(GamepadButtonMap.dpad.right, gamepad.dpad_right);

        gamepadFloatMap.put(GamepadTriggerMap.left, gamepad.left_trigger);
        gamepadFloatMap.put(GamepadTriggerMap.right, gamepad.right_trigger);

        gamepadFloatsMap.put(GamepadStickMap.left, new FloatTuple(gamepad.left_stick_x, gamepad.left_stick_y));
        gamepadFloatsMap.put(GamepadStickMap.right, new FloatTuple(gamepad.right_stick_x, gamepad.right_stick_y));
    }
    class FloatTuple{ //pretty sure there's another tuple class, but this is just temporary
        float x;
        float y;
        public FloatTuple(float X, float Y) {
            x=X;
            y=Y;
        }
    }
}
