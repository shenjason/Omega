package org.firstinspires.ftc.teamcode.util;

/**
 * ActionPress - Edge-detection utility for gamepad button presses.
 *
 * Ensures an action fires only once per button press (on the rising edge),
 * preventing repeated triggering while a button is held down.
 *
 * Usage example:
 *   ActionPress toggleTracking = new ActionPress(() -> robot.toggleMode());
 *   // In loop:
 *   toggleTracking.update(gamepad1.a);  // Action fires once when A is pressed
 */
public class ActionPress {
    /** The action to execute on button press */
    public Action pressAction;

    /** Tracks whether the button was already pressed (prevents re-triggering while held) */
    private boolean isActionPressed;

    public ActionPress(Action action){
        pressAction = action;
    }

    /**
     * Checks the button state and fires the action on the rising edge (press).
     * @param key Current button state (true = pressed, false = released)
     */
    public void update(boolean key){
        if (!key) isActionPressed = false;        // Button released — reset for next press
        if (key && !isActionPressed){
            pressAction.action();                  // Fire action on rising edge
            isActionPressed = true;
        }
    }

    /**
     * Checks the button state with an additional condition gate.
     * Action only fires if both the button is pressed AND the condition is true.
     * @param key       Current button state
     * @param condition Additional condition that must be true for the action to fire
     */
    public void update(boolean key, boolean condition){
        if (!key) isActionPressed = false;
        if (key && !isActionPressed && condition){
            pressAction.action();
            isActionPressed = true;
        }
    }
}
