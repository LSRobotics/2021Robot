// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;


/** Add your docs here. */
public class Statics {
    public enum CompetitionSelection{
        COMPETITION_1, COMPETITION_2, COMPETITION_3, COMPETITION_4, COMPETITION_5, TEST
    }
    public enum PartSelection{
        PART_1, PART_2, PART_3
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

    public static int navx = 10;

    public static double stickDeadzone = 0.1;

    public static double intakeSpeed = .35;
    public static double shooterSpeed = -.95;
    public static double indexClearSpeed = .2;

    public static int US_Maxbotix_Front = 3;
    public static int front_ir = 1;

    public static double cm_to_in = 0.049212598;
    public static double shooterVelocity = -19000;

    public static int pneumatic_intake_forward_channel = 2;
    public static int pneumatic_intake_backward_channel = 7;
    public static int pneumatic_climb_ratchet_forward_channel = 6;
    public static int pneumatic_climb_ratchet_backward_channel = 3;
    public static int pneumatic_drive_train_gear_shift_forward_channel = 0;
    public static int pneumatic_drive_train_gear_shift_backward_channel = 1;
    
    //Change the "COMPETITION_1" in the next line to whichever competition you need to run.
    static final public CompetitionSelection current_competition = CompetitionSelection.TEST;
    //Change the variable in order to select a part if applicable to respective challenges
    static final public PartSelection current_part = PartSelection.PART_1;


    //FR (1M): 77472
    //FL (1M): 77231
    //BL (1M): 77242
    //BR (1M): 77389
    //average: 77333.5

}
