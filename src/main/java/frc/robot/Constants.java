package frc.robot;

public final class Constants {
    //ALL UNITS ARE BASED AROUND FEET!! 
    // SO WE HAVE f, f/s, and f/s/s!!!!!

    public static final float DT = 1/50;

    public final class Drive {
        public static final float KS = 0; //Voltage required to overcome static friction
        public static final float KV = 1; //Voltage required to stay moving at 1 f/s
        public static final float KA = 1; //Voltage required to accelerate at 1 f/s/s

        public static final float MAX_ACC = 99999; //Maximum acceleration
        public static final float DRIVE_V_LIMIT = 12; //Voltage limit
    }

    public final class Arm {
        public static final float P = 1;
        public static final float I = 0;
        public static final float D = 0;
        public static final float ZEROING_VOLTAGE = 1; //Voltage for sending arm to start

    }

    public final class Controllers {
             // Gamepad axis
            public static final int kGamepadAxisLeftStickX = 3;
            public static final int kGamepadAxisLeftStickY = 4;
            public static final int kGamepadAxisRightStickX = 1;
            public static final int kGamepadAxisRightStickY = 2;
             //Todo: D-Pad?

            // Gamepad buttons
            public static final int kGamepadButtonX = 1;
            public static final int kGamepadButtonA = 2; 
            public static final int kGamepadButtonB = 3; 
            public static final int kGamepadButtonY = 4; 
            public static final int kGamepadButtonLB = 5;
            public static final int kGamepadButtonRB = 6;
            public static final int kGamepadButtonLT = 7;
            public static final int kGamepadButtonRT = 8;
            public static final int kGamepadButtonBack = 9;
            public static final int kGamepadButtonStart = 10;
    }
}
