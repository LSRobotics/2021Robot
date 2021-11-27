// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;


/** Add your docs here. */
public class Statics {
    public static int Wheel_FrontLeft = 4;
    public static int Wheel_BackLeft = 3;
    public static int Wheel_FrontRight = 1;
    public static int Wheel_BackRight = 2;

    public static int intake_front = 8;
    public static int intake_toShooter = 7;
    public static int intake_bottom = 6;
    public static int intake_top = 5;

    public static int shooter = 9;

    public static int navx = 10;
    public final static double PID_GYRO_TOLERANCE = 2;
    public final static double GYRO_P = .2;
    public final static double GYRO_I = 0;
    public final static double GYRO_D = .2;
    public final static double GYRO_F = 0;

    public static int climb_motor_id = 11;

    public static double stickDeadzone = 0.1;

    public static double intakeSpeed = .35;
    public static double shooterSpeed = -.95;
    public static double indexClearSpeed = .2;
    public static double moveSpeed = 0.5;

    public static double lowModeModifier = 0.4;
    public static double highModeModifier = 0.80;

    public static int ultrasonic = 3; //NOT IN 3
    public static int front_ir = 1;

    public static double cm_to_in = 0.049212598;
    public static double shooterVelocity = -19000;
    public static double intakeIRThreshold = 0.7;


    public static int pneumatic_intake_forward_channel = 2;
    public static int pneumatic_intake_backward_channel = 7;
    public static int pneumatic_climb_ratchet_forward_channel = 6;
    public static int pneumatic_climb_ratchet_backward_channel = 3;
    public static int pneumatic_drive_train_gear_shift_forward_channel = 0;
    public static int pneumatic_drive_train_gear_shift_backward_channel = 1;

    //FR (1M): 77472
    //FL (1M): 77231
    //BL (1M): 77242
    //BR (1M): 77389
    //average: 77333.5 == 3 feet
    //AV (1Foot): 25777.8333
    //AV (1Inch): 2148.15278

    public static double inchToSensorUnits = 2148.15278;

}
