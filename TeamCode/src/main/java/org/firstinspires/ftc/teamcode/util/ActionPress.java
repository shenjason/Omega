package org.firstinspires.ftc.teamcode.util;

public class ActionPress {
    public Action pressAction;

    private boolean isActionPressed;
    public ActionPress(Action action){
        pressAction = action;
    }

    public void update(boolean key){
        if (!key) isActionPressed = false;
        if (key && !isActionPressed){
            pressAction.action();
            isActionPressed = true;
        }
    }

    public void update(boolean key, boolean condition){
        if (!key) isActionPressed = false;
        if (key && !isActionPressed && condition){
            pressAction.action();
            isActionPressed = true;
        }
    }
}