package frc.robot;

public final class Constants {
    // UNITS           = Name (Abbr)
    //-----------------|-------------
    // LENGTH/DISTANCE = Feet (f)
    // VELOCITY/SPEED  = Feet per Second (f/s)
    // ACCELERATION    = Feet per Second per Second (f/s/s)
    // ANGLE           = Degrees or Rotations- will be labeled
    // ELECTRIC POWER  = Volts (v)

    public static final float DT = 1/50;

    public final class Drive {
        public static final float KS = 0; //Voltage required to overcome static friction
        public static final float KV = 1; //Voltage required to stay moving at 1 f/s
        public static final float KA = 1; //Voltage required to accelerate at 1 f/s/s

        public static final float MAX_ACC = 99999; //Maximum acceleration
        public static final float DRIVE_V_LIMIT = 12; //Voltage limit
    }

    public final class Arm {

        public static final float ENCODER_COUNTS_PER_REV = 4096; //
        public static final float RATIO = 400 / 1 ;//Gear ratio from falcon to arm; how many falcon rotations per arm rotation

        public static final float ENCODER_COUNTS_PER_ARM_REV = ENCODER_COUNTS_PER_REV * RATIO;
        public static final float ENCODER_COUNTS_TO_ARM_DEGS = 360/ENCODER_COUNTS_PER_ARM_REV;


        public static final float P = 1;
        public static final float I = 0;
        public static final float D = 0;
        public static final float ZEROING_VOLTAGE = 12/2; //Voltage for sending arm to start

        //Reasonable estimates from CAD
        //Need to be tuned
        //Note: Relative to limit switch hit position!
        public static final float ANGLE_HI   = 100; //Angle required to reach highest row on a Grid
        public static final float ANGLE_MID  = 85;  //Angle required to reach middle row on a Grid
        public static final float ANGLE_LO   = 30;  //Angle that gets us the closest to the ground (May go unused)
        public static final float ANGLE_HOLD = 0;   //Angle that gets the arm all the way inside the frame perimeter

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
