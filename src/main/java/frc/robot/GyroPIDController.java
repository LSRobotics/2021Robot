package main.java.frc.robot;

import edu.wpi.first.wpilibj.*;

public class GyroPIDController extends PIDController {

    public GyroPIDController(double kP, double kI, double kD, double kF, PIDSource source, PIDOutput output) {
        super(kP, kI, kD, kF, source, output);
    }

    public void calculate() {
        super.calculate();
    }

}