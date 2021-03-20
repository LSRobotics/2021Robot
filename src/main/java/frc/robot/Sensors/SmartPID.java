package main.java.frc.robot.Sensors;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.controller.PIDController;

public class SmartPID extends PIDController {

    ArrayList<Double> history = new ArrayList<Double>(); //FIXME: A potential source of crash due to limited RAM

    Timer timer = new Timer(/*"SmartPID Timer"*/);
    double oscilateCutOff = 0.1;
    boolean isActionDone = false;

    public SmartPID(double Kp, double Ki, double Kd) {
        super(Kp,Ki,Kd);
        timer.start();
    }

    public double next(double reading) {
        
        double result = calculate(reading);

        if((timer.get() * 1000) > 100) {
            history.add(result);
            timer.start();
        }

        if(!isActionDone && history.size() > 4 && history.size() % 2 == 0) {

            double max = 0;

            for(int i = history.size() - 5; i < history.size(); ++i) {
                double val = Math.abs(history.get(i));

                if(val > max) {
                    max = val;
                }
            }

            if(max < oscilateCutOff) {
                isActionDone = true;
                history.clear();
            }
        }

        return result;

    }

    public void setCutOff(double oscilateCutOff) {
        this.oscilateCutOff = Math.abs(oscilateCutOff);
    }

    /**
     * I don't see why I even need this method, but whatever
     */
    public void reset() {
        isActionDone = false;
    }

    public boolean isActionDone() {
        return isActionDone;
    }
}