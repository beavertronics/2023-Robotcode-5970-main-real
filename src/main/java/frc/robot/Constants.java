package frc.robot;

public final class Constants {
    // UNITS            = Name (Abbr)
    //------------------|-------------
    // Distance         = Inches (in)
    // Speed/Velocity   = Inches per Second (in/s)
    // Acceleration     = Inches per Second per Second (in/s/s)
    // Angles/rotations = Revolutions (revs)
    // 
    // Electricity uses Volts, Amps and Watts


    public static final double DT = 0.02;

    public final class Drive {
        public static final double P  = 20;
        public static final double D = 0;
        
        public static final double KS = 0.24571 / 12; //Voltage required to overcome static friction
        //v
        public static final double KV = 0.57682 / 12; //Voltage required to stay moving at 1 f/s
        //v/(in/s)
        public static final double KA = 0.07    / 12;//0.24682; //Voltage required to accelerate at 1 f/s/s
        //v/(in/s/s)
        public static final float MAX_ACC = 2; //Maximum acceleration
        public static final float DRIVE_V_LIMIT = 12; //Voltage limit

        //All of the above apply to BOTH teleop AND AUTONOMOUS!
        //TELE_SPEED_MULT only applies to teleop.

        public static final double TELE_SPEED_MULT = 7 * 12; //Fast!
        //The speed range of the bot should be from 0f/s to TELE_SPEED_MULT in/s
        // But problems...

        public static final double LOW_GEAR  = 1.0/24.0;
        public static final double HIGH_GEAR = 15.0/136.0;
        public static final double WHEEL_DIA = 7.650;
        public static final double WHEEL_CIRC = WHEEL_DIA * Math.PI;
        public static final double LOW_GEAR_REVS_TO_INCHES  = LOW_GEAR  * WHEEL_CIRC;
        public static final double HIGH_GEAR_REVS_TO_INCHES = HIGH_GEAR * WHEEL_CIRC;

        public static final double INCHES_TO_LOW_GEAR_REVS = 1/LOW_GEAR_REVS_TO_INCHES;
        public static final double INCHES_TO_HIGH_GEAR_REVS = 1/HIGH_GEAR_REVS_TO_INCHES;
    }

    public final class Arm {

        //public static final double ENCODER_COUNTS_PER_REV = 2048.0; //
        //public static final double RATIO = 1.0 / 400.0 ;//Gear ratio from falcon to arm; how many falcon rotations per arm rotation

        //public static final double ENCODER_COUNTS_PER_ARM_REV = ENCODER_COUNTS_PER_REV / RATIO;
        //public static final double ENCODER_COUNTS_TO_ARM_DEGS = 1.0 / (ENCODER_COUNTS_PER_ARM_REV / 360.0);

        //public static final double ARM_V_LIMIT = 10; //Limit output of arm PID via clamping

       // public static final double P = 1;
        //public static final float I = 0;
        //public static final float D = 0;
        //public static final float ZEROING_VOLTAGE = -4; //Voltage for sending arm to start

        //Reasonable estimates from CAD
        //Need to be tuned
        //Note: Relative to limit switch hit position!
        public static final float ANGLE_HI   = 122; //Angle required to reach highest row on a Grid
        public static final float ANGLE_MID  = 92;  //Angle required to reach middle row on a Grid
        public static final float ANGLE_LO   = 32;  //Angle that gets us the closest to the ground (May go unused)
        public static final float ANGLE_HOLD = 0;   //Angle that gets the arm all the way inside the frame perimeter

    }
    public final class Auto {

        public static final double GYRO_ANGLE_ADJUST = 0; 
        //adjust GYRO_ANGLE_ADJUST so that Gyro pitch + GYRO_ANGLE_ADJUST is zero when gyro is level

        public static final double LEVELING_P = 1;
        public static final double LEVELING_I = 0;
        public static final double LEVELING_D = 0;

        public static final double LEVELING_ACCURACY = 1; //Will not stop until it is within one degree of level

        public static final double POSITION_ACCURACY = 0.25; //Quarter inch


        public static final double TRAVERSAL_SPEED = 3; //Auto will try to run at 3 inches?? per second??
        public static final double STAION_APPROACH_SPEED = TRAVERSAL_SPEED/2; //Auto will go slowly when approaching charge station

        public static final double SCORING_DRIVE_TIME = 0.7;
        public static final double LEAVING_DRIVE_TIME = 1.5;

        public static final double LEVELING1_APPROACH_TIME = 1; //How long to get onto platform
        public static final double LEVELING2_EXIT_TIME = 1; //How long will the robot take to drive all the way over the charge station and leave the community for mobility points
        public static final double LEVELING2_REVERSE_APPROACH_TIME = 1; //How long does the robot need to drive backwards after leaving the community to get partially back on the charge station
    }

    public final class LogitechF130Controller {
        //Largely borrowed from http://controls.coderedrobotics.com/programminglessons/4.html
        //TODO: Refactor this into a properly integrated class, or borrow someone else's properly integrated F130 gamepad class.
        //Gamepad axes (All vary from -1 to 1)
        public static final int kAxisLeftStickX = 0;
        public static final int kAxisLeftStickY = 1;
        public static final int kAxisLT = 2; //Triggers are analog (they are below bumper buttons)
        public static final int kAxisRT = 3;
        public static final int kAxisRightStickX = 4;
        public static final int kAxisRightStickY = 5;

        //D-Pad works differently and we don't even really need it so I didn't include it

        // Gamepad buttons (clickysticks included)
        public static final int kButtonA = 1; 
        public static final int kButtonB = 2; 
        public static final int kButtonX = 3;
        public static final int kButtonY = 4; 

        public static final int kButtonLB = 5; //Bumpers are different from triggers and are not analog
        public static final int kButtonRB = 6;

        public static final int kButtonBack = 7;
        public static final int kButtonStart = 8;

        public static final int kButtonLClick = 9;
        public static final int kButtonRClick = 10;
    }
}
