package frc.robot;

public final class Constants {
    // UNITS           = Name (Abbr)
    //-----------------|-------------
    // LENGTH/DISTANCE = Feet (f)
    // VELOCITY/SPEED  = Feet per Second (f/s)
    // ACCELERATION    = Feet per Second per Second (f/s/s)
    // ANGLE           = Degrees or Rotations- will be labeled
    // ELECTRIC POWER  = Volts (v)

    public static final double DT = 0.02;

    public final class Drive {
        public static final double KS = 0.24571; //Voltage required to overcome static friction
        public static final double KV = 1.8927; //Voltage required to stay moving at 1 f/s
        public static final double KA = 0.80986; //Voltage required to accelerate at 1 f/s/s

        public static final float MAX_ACC = 99999; //Maximum acceleration
        public static final float DRIVE_V_LIMIT = 12; //Voltage limit

        //All of the above apply to BOTH teleop AND AUTONOMOUS!
        //TELE_SPEED_MULT only applies to teleop.

        public static final double TELE_SPEED_MULT = 5; //Fast!

        public static final double ROTATIONS_PER_INCH = 576.79641; //IN LOW GEAR ONLY!!
    }

    public final class Arm {

        public static final float ENCODER_COUNTS_PER_REV = 2048; //
        public static final float RATIO = 400 / 1 ;//Gear ratio from falcon to arm; how many falcon rotations per arm rotation

        public static final double ENCODER_COUNTS_PER_ARM_REV = ENCODER_COUNTS_PER_REV * RATIO;
        public static final double ENCODER_COUNTS_TO_ARM_DEGS = 360/ENCODER_COUNTS_PER_ARM_REV;


        public static final double P = 1.5;
        public static final float I = 0;
        public static final float D = 0;
        public static final float ZEROING_VOLTAGE = -12/2; //Voltage for sending arm to start

        //Reasonable estimates from CAD
        //Need to be tuned
        //Note: Relative to limit switch hit position!
        public static final float ANGLE_HI   = 100; //Angle required to reach highest row on a Grid
        public static final float ANGLE_MID  = 85;  //Angle required to reach middle row on a Grid
        public static final float ANGLE_LO   = 30;  //Angle that gets us the closest to the ground (May go unused)
        public static final float ANGLE_HOLD = 0;   //Angle that gets the arm all the way inside the frame perimeter

    }
    public final class Auto {
        public static final double STATION_DETECTION_TILT = 10; 
        //How far the robot has to tilt before it can assume it's at least partially on the charge station

        public static final double STATION_EXIT_DETECTION_TILT = 1;
        //How close to level the robot has to be before it can assume it's mostly off the charge station

        public static final double STATION_EXIT_EXTRA_DIST = 6;
        //How many extra inches to travel after hitting STATION_EXIT_DETECTION_TILT to be fully off of the station and out of the community


        public static final double LEAVE_DIST = 5 * 12;
        //How many inches to travel to fully leave the COMMUNITY.

        public static final double TRAVERSAL_SPEED = 3.5; //Auto will try to run at 3 feet per second.
         //Speed while leveling the charge station is automagically adjusted by PD controller

        public static final double LEVELING_TOLERANCE = 0.5; //How close the robot will try to get to perfectly level

        //The tolerance on drive station counting as level is about 2 + 1/2 degrees
        //Gyro accuracy is about 1 degree and it does drift pretty bad.
    }

    public final class LogitechF130Controller {
        //Largely borrowed from http://controls.coderedrobotics.com/programminglessons/4.html
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
