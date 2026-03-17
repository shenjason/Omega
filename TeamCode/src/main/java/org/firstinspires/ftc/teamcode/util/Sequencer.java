package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * Sequencer - Executes a timed sequence of actions with optional conditions.
 *
 * Used for the shooter's fire sequence (open gate -> reverse intake -> forward intake -> close gate)
 * and other multi-step robot operations that need precise timing.
 *
 * Each step has:
 *   - An Action (lambda/method reference to execute)
 *   - A timing delay (seconds to wait before executing this step)
 *   - An optional Condition (must return true before the step executes)
 *
 * The sequencer advances through steps sequentially. Call start() to begin and
 * update() every loop cycle to advance the sequence.
 *
 * currentIndex = -1 means the sequence is inactive (not running).
 */
public class Sequencer {

    public List<Condition> conditions;

    public List<Action> actions;

    /** Delay in seconds before each action executes (measured from when the previous step completed) */
    public List<Double> timings;

    /** Current step index in the sequence (-1 = inactive/complete) */
    public int currentIndex = 0;

    Timer timer;


    /**
     * Creates a sequencer with actions and timings (all conditions default to true).
     * @param actions List of actions to execute in order
     * @param timings List of delays (seconds) before each action
     */
    public Sequencer(List<Action> actions, List<Double> timings){
        this.actions = actions;
        this.timings = timings;
        this.conditions = new ArrayList<>();
        timer = new Timer();

        // Default all conditions to always-true
        for (int i=0; i<actions.size(); i++){
            this.conditions.add(Sequencer.defaultCondition());
        }

        reset();
    }

    /**
     * Creates a sequencer with actions, timings, and custom conditions.
     * @param actions    List of actions to execute in order
     * @param timings    List of delays (seconds) before each action
     * @param conditions List of conditions that must be true before each action executes
     */
    public Sequencer(List<Action> actions, List<Double> timings, List<Condition> conditions){
        this.actions = actions;
        this.conditions = conditions;
        this.timings = timings;
        timer = new Timer();

        reset();
    }


    /**
     * Advances the sequence by one step if the timing delay has elapsed
     * and the condition for the current step is met.
     * Must be called every loop cycle while the sequence is running.
     */
    public void update(){
        if (currentIndex == -1) return; // Sequence not active
        if (timer.getElapsedTimeSeconds() > timings.get(currentIndex) && conditions.get(currentIndex).condition()){
            actions.get(currentIndex).action(); // Execute the current action
            currentIndex ++;
            timer.resetTimer(); // Reset timer for the next step's delay
        }
        if (currentIndex >= this.actions.size()) currentIndex = -1; // Sequence complete
    }

    /** Starts (or restarts) the sequence from the beginning */
    public void start(){
        currentIndex = 0;
        timer.resetTimer();
    }

    /** Returns a condition that always evaluates to true (default for all steps) */
    public static Condition defaultCondition(){
        return () -> true;
    }

    /** Returns an action that does nothing (placeholder) */
    public static Action noneAction() {return Sequencer::none; }

    static void none() {}

    /** Stops the sequence and resets to inactive state */
    public void reset(){
        currentIndex = -1;
        timer.resetTimer();
    }
}
