package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

import java.util.ArrayList;
import java.util.List;

public class Sequencer {

    public List<Condition> conditions;

    public List<Action> actions;

    public List<Double> timings;

    public int currentIndex = 0;

    Timer timer;


    public Sequencer(List<Action> actions, List<Double> timings){
        this.actions = actions;
        this.timings = timings;
        this.conditions = new ArrayList<>();
        timer = new Timer();


        for (int i=0; i<actions.size(); i++){
            this.conditions.add(Sequencer.defaultCondition());
        }

        reset();
    }

    public Sequencer(List<Action> actions, List<Double> timings, List<Condition> conditions){
        this.actions = actions;
        this.conditions = conditions;
        this.timings = timings;
        timer = new Timer();

        reset();
    }


    public void update(){
        if (currentIndex == -1) return;
        if (timer.getElapsedTimeSeconds() > timings.get(currentIndex) && conditions.get(currentIndex).condition()){
            actions.get(currentIndex).action();
            currentIndex ++;
            timer.resetTimer();
        }
        if (currentIndex >= this.actions.size()) currentIndex = -1;
    }

    public void start(){
        currentIndex = 0;
        timer.resetTimer();
    }

    public static Condition defaultCondition(){
        return () -> true;
    }
    public static Action noneAction() {return Sequencer::none; }

    static void none() {}
    public void reset(){
        currentIndex = -1;
        timer.resetTimer();
    }
}
