// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

public class Statics {

    public static final int TALON_FX_DRIVE_FRONTRIGHT_ID = 1;
    public static final int TALON_FX_DRIVE_BACKRIGHT_ID = 2;
    public static final int TALON_FX_DRIVE_BACKLEFT_ID = 3;
    public static final int TALON_FX_DRIVE_FRONTLEFT_ID = 4;

    public static final int TALON_FX_INTAKE_TOP_ID = 5;
    public static final int TALON_FX_INTAKE_BOTTOM_ID = 6;
    public static final int TALON_FX_INTAKE_TOSHOOTER_ID = 7;
    public static final int TALON_FX_INTAKE_FRONT_ID = 8;

    public static final int TALON_FX_SHOOTER_ID = 9;

    public static final int NAVX_ID = 10;
    public static final double PID_GYRO_TOLERANCE = 2;
    public static final double GYRO_P = .2;
    public static final double GYRO_I = 0;
    public static final double GYRO_D = .2;
    public static final double GYRO_F = 0;

    public static final int TALON_FX_CLIMB_ID = 11;

    public static final double STICK_DEADZONE = 0.1;

    //All go from 0 to 1
    public static final double INTAKE_SPEED = .35;
    public static final double SHOOTER_SPEED = -.95;
    public static final double INDEX_CLEAR_SPEED = .2;
    public static final double MOVE_SPEED = 0.5;

    public static final double lowModeModifier = 0.4;
    public static final double highModeModifier = 0.8;

    public static final int ULTASONIC_ID = 3; //NOT IN 3
    public static final int IR_FRONT_ID = 1;

    public static final double CM_TO_IN = 0.049212598;
    public static final double SHOOTER_VELOCITY = -19000;
    public static final double INTAKE_IR_THRESHOLD = 0.7;


    public static final int PNEUMATIC_INTAKE_FORWARD_CHANNEL = 2;
    public static final int PNEUMATIC_INTAKE_BACKWARD_CHANNEL = 7;
    public static final int PNEUMATIVE_CLIMB_RATCHET_FORWARD_CHANNEL = 6;
    public static final int PNEUMATIVE_CLIMB_RATCHET_BACKWARD_CHANNEL = 3;
    public static final int PNEUMATIC_DRIVETRAIN_GEARSHIFT_FORWARD_CHANNEL = 0;
    public static final int PNEUMATIC_DRIVETRAIN_GEARSHIFT_BACKWARD_CHANNEL = 1;

    //FR (1M): 77472
    //FL (1M): 77231
    //BL (1M): 77242
    //BR (1M): 77389
    //average: 77333.5 == 3 feet
    //AV (1Foot): 25777.8333
    //AV (1Inch): 2148.15278

    public static double inchToSensorUnits;

}
