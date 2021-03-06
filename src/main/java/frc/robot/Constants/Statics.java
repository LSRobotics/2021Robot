// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;


/** Add your docs here. */
public class Statics {
    public enum CompetitionSelection{
        COMPETITION_1, COMPETITION_2, COMPETITION_3, COMPETITION_4, COMPETITION_5
    }
    public static int Wheel_FrontLeft = 1;
    public static int Wheel_BackLeft = 2;
    public static int Wheel_FrontRight = 3;
    public static int Wheel_BackRight = 4;

    public static int intake_front = 8;
    public static int intake_toShooter = 7;
    public static int intake_bottom = 6;
    public static int intake_top = 5;

    public static int shooter = 9;

    public static double stickDeadzone = 0.1;

    public static double intakeSpeed = .35;
    public static double shooterSpeed = .95;

    public static int US_Maxbotix_Front = 3;

    public static double cm_to_in = 0.049212598;

    public static int pneumatic1_forward_channel = 0;
    public static int pneumatic1_back_channel = 1;
    public static int pneumatic2_forward_channel = 2;
    public static int pneumatic2_back_channel = 3;
    public static int pneumatic3_forward_channel = 4;
    public static int pneumatic3_back_channel = 5;
    
    //Change the "COMPETITION_1" in the next line to whichever competition you need to run.
    static final public CompetitionSelection current_competition = CompetitionSelection.COMPETITION_1;


}
