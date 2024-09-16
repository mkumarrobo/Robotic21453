package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "AUTON BLUE FRONT", group = "Iterative Opmode")
public class Auton_Blue_Front extends LinearOpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize DcMotors.
    private DcMotor motor_FRM = null;
    private DcMotor motor_FLM = null;
    private DcMotor motor_RLM = null;
    private DcMotor motor_RRM = null;
    private DcMotor motor_ELBOW = null;
    private DcMotor motor_XTND = null;
    private Servo L_grab = null;
    private Servo R_grab = null;
    private ElapsedTime runtime = new ElapsedTime();
    DistanceSensor Dist_OR;
    DistanceSensor Dist_OL;
    DistanceSensor Dist_IR;
    DistanceSensor Dist_IL;
    double drive = 0;
    double strafe = 0;
    double twist = 0;
    int ELBOW_UpPos = 1000;
    int XTND_SetPos;
    double default_drive_power = 0.3;

    //    IMU parameters
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;

    //    Other sensor parameters
    double desired_distance = 20;
    double Current_Heading;
    double INCH2ENC = 153.6;
    double INCH2ENC_ST = 671;
    int Pixel_Pos = 3;

    // Linear variables for XTND and DIST relationship
    double m =18.89;
    double b = 737.67;

    //    Camera parameters
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "Blue_Crown_3.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)

    private static final String[] LABELS = {
            "Blank", "Red Crown"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        // Initialize DC Motors
        motor_FLM = hardwareMap.get(DcMotor.class, "FLM");
        motor_FRM = hardwareMap.get(DcMotor.class, "FRM");
        motor_RLM = hardwareMap.get(DcMotor.class, "RLM");
        motor_RRM = hardwareMap.get(DcMotor.class, "RRM");
        motor_ELBOW = hardwareMap.get(DcMotor.class, "ELBOW");
        motor_XTND = hardwareMap.get(DcMotor.class, "XTND");

        // Initialize Servo Motors
        L_grab = hardwareMap.get(Servo.class, "Grapple_Left");
        R_grab = hardwareMap.get(Servo.class, "Grapple_Right");

        // Initialize Sensors
        Dist_OR = hardwareMap.get(DistanceSensor.class, "DIST_OR");
        Dist_OL = hardwareMap.get(DistanceSensor.class, "DIST_OL");
        Dist_IR = hardwareMap.get(DistanceSensor.class, "DIST_IR");
        Dist_IL = hardwareMap.get(DistanceSensor.class, "DIST_IL");
        imu = hardwareMap.get(IMU.class, "IMU");
        logoFacingDirectionPosition = 2; // Forward
        usbFacingDirectionPosition = 0; // Up

        updateOrientation();
        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;

        // Setup lifting DC motors for drive by encoder
//        motor_ELBOW.setDirection(DcMotorSimple.Direction.REVERSE);


        motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_ELBOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_XTND.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_ELBOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_XTND.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initTfod();
        double ez = .65;
        double Air = 0;
        imu.resetYaw();
        double Set_Heading;


//        motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        L_grab_close();
        R_grab_close();

        runtime.reset();
//        while ((runtime.seconds() <= 10)) {
//            telemetryTfod();
////            if (Min_Dist[0] > Dist_OL.getDistance(DistanceUnit.CM)) {
////                Min_Dist[0] = Dist_OL.getDistance(DistanceUnit.CM);
////            }
////            if (Min_Dist[1] > Dist_IL.getDistance(DistanceUnit.CM)) {
////                Min_Dist[1] = Dist_IL.getDistance(DistanceUnit.CM);
////            }
////            if (Min_Dist[2] > Dist_IR.getDistance(DistanceUnit.CM)) {
////                Min_Dist[2] = Dist_IR.getDistance(DistanceUnit.CM);
////            }
////            if (Min_Dist[3] > Dist_OR.getDistance(DistanceUnit.CM)) {
////                Min_Dist[3] = Dist_OR.getDistance(DistanceUnit.CM);
////            }
//        }
//        telemetry.addData("Pixel Position = ", "%7d", Pixel_Pos);
//        telemetry.update();

        Current_Heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


//        if (Min_Dist[0] < 500) {
//            Pixel_Pos = 1;
//        }
//        else if (Min_Dist[1] + Min_Dist[2] < 1200) {
//            Pixel_Pos = 2;
//        }


        waitForStart();
//        Air = ((Dist_OR.getDistance(DistanceUnit.CM) + (Dist_OL.getDistance(DistanceUnit.CM)))) / 2;
        runtime.reset();
        No_St_Drive_Enc(5, default_drive_power);
        while ((runtime.seconds() <= 5)) {
            telemetryTfod();
        }
        telemetry.addData("Pixel Position = ", "%7d", Pixel_Pos);
        telemetry.update();
        if (Pixel_Pos == 3) {
            No_St_Drive_Enc(25, default_drive_power);
            St_Enc(2.5);
            R_grab.setPosition(0.7);
            sleep(500);
            No_St_Drive_Enc(-5, default_drive_power);
            R_grab.setPosition(1);
            St_Enc(3);
            No_St_Drive_Enc(36, default_drive_power);
            Twist_IMU(-90);

            No_St_Drive_Enc(120, .6);
            ELBOW_Go(ELBOW_UpPos);

            XTND_SetPos = (int) (m * Dist_OL.getDistance(DistanceUnit.CM) + b);
            if (XTND_SetPos>849) {
                XTND_SetPos = 849;
            }
            XTND_Go(XTND_SetPos);
            L_grab_open();
            sleep(100);
            XTND_Go(0);
            L_grab_close();
            St_Enc(5);
            ELBOW_Go(0);

        }



        else if (Pixel_Pos == 2) {
            No_St_Drive_Enc(30, default_drive_power);
            R_grab.setPosition(0.7);//        Right gripper open
            sleep(500);
            No_St_Drive_Enc(-8, default_drive_power);
            R_grab.setPosition(1);//        Right gripper close
            St_Enc(3);
            No_St_Drive_Enc(33, default_drive_power);
            Twist_IMU(-90);
            No_St_Drive_Enc(104, 0.5);
            St_Enc(-9);

            ELBOW_Go(ELBOW_UpPos);

            XTND_SetPos = (int) (m * Dist_OL.getDistance(DistanceUnit.CM) + b);
            if (XTND_SetPos>849) {
                XTND_SetPos = 849;
            }
            XTND_Go(XTND_SetPos);
            L_grab_open();
            sleep(100);
            XTND_Go(0);
            L_grab_close();
            St_Enc(5);
            ELBOW_Go(0);

        }

        else {
            No_St_Drive_Enc(20, default_drive_power);
            Twist_IMU(-50);
            No_St_Drive_Enc(11, default_drive_power);
            R_grab.setPosition(0.7);//        Right gripper open
            sleep(1000);
            No_St_Drive_Enc(-10, default_drive_power);
            R_grab.setPosition(1);//        Right gripper close
            runtime.reset();
            Twist_IMU(0);
            No_St_Drive_Enc(50, default_drive_power);
            Twist_IMU(-90);
            No_St_Drive_Enc(90, 1);
            sleep(500);
            St_Enc(8);
            ELBOW_Go(ELBOW_UpPos);

            XTND_SetPos = (int) (m * Dist_OL.getDistance(DistanceUnit.CM) + b);
            if (XTND_SetPos>849) {
                XTND_SetPos = 849;
            }
            XTND_Go(XTND_SetPos);
            L_grab_open();
            sleep(100);
            XTND_Go(0);
            L_grab_close();
            St_Enc(5);
            ELBOW_Go(0);

        }

//        sleep(1000);
//        telemetry.addData("Dr Distance", "%.3f", GetAvgDrDist());
//        telemetry.update();

//        mecanum_drive(0, 0.3, 0);





//        telemetry.addData("Dr Distance", "%.3f", GetAvgDrDist());
//        telemetry.update();
//

//        sleep(1000);
//
//






        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        //double max = Math.abs(speeds[0]);
        //for (int i = 0; i < speeds.length; i++) {
        //    if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        //}

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        //if (max > 1) {
        //    for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        //}


        // apply the calculated values to the motors.
        // Loop until stop requested

        while (!isStopRequested()) {

            if (orientationIsValid) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            }
            else {
                telemetry.addData("Error", "Selected orientation on robot is invalid");
            }
            telemetry.update();
        }


    }
    private void No_St_Drive_Enc(double Distance, double pwr) {
        Current_Heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        ResetDrEncoders();
        if (Distance > 0){
            while (GetAvgDrDist() < Distance){
                drive = pwr;
                twist = (Current_Heading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 20;
                mecanum_drive(drive, 0, twist);

            }
        }
        else if (Distance < 0){
            while (GetAvgDrDist() > Distance){
                drive = -pwr;
                twist = (Current_Heading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 20;
                mecanum_drive(drive, 0, twist);

            }
        }
//            telemetry.addData("Enc FLM", "%7d", motor_FLM.getCurrentPosition());
//            telemetry.addData("Enc FRM", "%7d", motor_FRM.getCurrentPosition());
//            telemetry.addData("Enc RLM", "%7d", motor_RLM.getCurrentPosition());
//            telemetry.addData("Enc RRM", "%7d", motor_RRM.getCurrentPosition());
//            telemetry.update();
        mecanum_drive(0, 0, 0);
    }

    private void St_Drive_Enc(double Distance) {
        Current_Heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        ResetDrEncoders();
        if (Distance > 0){

            while (GetAvgDrDist() < Distance){
                drive = 0.3;
                if (Dist_OL.getDistance(DistanceUnit.CM) < 50 || Dist_OR.getDistance(DistanceUnit.CM) < 50) {
                    strafe = 5 * (1 / Dist_OR.getDistance(DistanceUnit.CM) - 1 / Dist_OL.getDistance(DistanceUnit.CM));
                    if (strafe > 0.5) {
                        strafe = 0.5;
                    }
                    if (strafe < -0.5) {
                        strafe = -0.5;
                    }
//                twistdelta = strafedelta * 0.1;
                }
                else {
                    strafe = 0;
                }
                twist = (Current_Heading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 20;
                mecanum_drive(drive, strafe, twist);
            }
        }
        else if (Distance < 0){
            telemetry.addData("Dr Distance", "%.3f", GetAvgDrDist());
            telemetry.update();
            while (GetAvgDrDist() > Distance){
                drive = -0.3;
                twist = (Current_Heading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 20;
                mecanum_drive(drive, 0, twist);
            }
        }
        telemetry.addData("Enc FLM", "%7d", motor_FLM.getCurrentPosition());
        telemetry.addData("Enc FRM", "%7d", motor_FRM.getCurrentPosition());
        telemetry.addData("Enc RLM", "%7d", motor_RLM.getCurrentPosition());
        telemetry.addData("Enc RRM", "%7d", motor_RRM.getCurrentPosition());
        telemetry.update();
        mecanum_drive(0, 0, 0);
    }

    private void St_Enc(double Distance) {
        Current_Heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        ResetDrEncoders();
//        telemetry.addData("Strafing actual dist", "%.2f", GetAvgStDist());
//        telemetry.update();
        if (Distance > 0){
            while (GetAvgStDist() < Distance){
                strafe = 0.3;
                twist = (Current_Heading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 20;
//                telemetry.addData("Strafing actual dist", "%.2f", GetAvgStDist());
//                telemetry.addData("Strafing now to", "%.2f", strafe);
//            telemetry.update();
                mecanum_drive(0, strafe, twist);
            }
        }
        else if (Distance < 0){
            while (GetAvgStDist() > Distance){
                strafe = -0.3;
                twist = (Current_Heading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 20;
                mecanum_drive(0, strafe, twist);
            }
        }
//            telemetry.addData("Enc FLM", "%7d", motor_FLM.getCurrentPosition());
//            telemetry.addData("Enc FRM", "%7d", motor_FRM.getCurrentPosition());
//            telemetry.addData("Enc RLM", "%7d", motor_RLM.getCurrentPosition());
//            telemetry.addData("Enc RRM", "%7d", motor_RRM.getCurrentPosition());
//            telemetry.update();
        mecanum_drive(0, 0, 0);
    }

    private void Twist_IMU(double Heading) {
        if (Heading > imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < Heading){
                twist = .4;
                mecanum_drive(0, 0, twist);
            }
        }
        else {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > Heading){
                twist = -.4;
                mecanum_drive(0, 0, twist);
            }
        }
//            telemetry.addData("Enc FLM", "%7d", motor_FLM.getCurrentPosition());
//            telemetry.addData("Enc FRM", "%7d", motor_FRM.getCurrentPosition());
//            telemetry.addData("Enc RLM", "%7d", motor_RLM.getCurrentPosition());
//            telemetry.addData("Enc RRM", "%7d", motor_RRM.getCurrentPosition());
//            telemetry.update();
        mecanum_drive(0, 0, 0);
    }

    private void mecanum_drive(double dr, double st, double tw) {
//        telemetry.addData("Drive", "%.3f", dr);
//        telemetry.addData("Strafe", "%.3f", st);
//        telemetry.addData("Twist", "%.3f", tw);
//        telemetry.update();
        dr*=-1;
        double[] speeds = {
                (dr + st + tw),
                (dr - st - tw),
                (dr - st + tw),
                (dr + st - tw)
        };
        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] *= 0.5/max;
        }

        motor_FLM.setPower(speeds[0]);
        motor_FRM.setPower(-speeds[1]);
        motor_RLM.setPower(speeds[2]);
        motor_RRM.setPower(-speeds[3]);
    }

    private void XTND_Go(int SetPos) {
        if (motor_XTND.getCurrentPosition() < SetPos) {
            while (motor_XTND.getCurrentPosition() < SetPos) {
                motor_XTND.setTargetPosition((int) SetPos);
                motor_XTND.setPower(1);
                motor_XTND.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else {
            while (motor_XTND.getCurrentPosition() > SetPos) {
                motor_XTND.setTargetPosition((int) SetPos);
                motor_XTND.setPower(1);
                motor_XTND.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

    }

    private void ELBOW_Go(int SetPos) {
        if (motor_ELBOW.getCurrentPosition() < SetPos) {
            while (motor_ELBOW.getCurrentPosition() < SetPos) {
                motor_ELBOW.setTargetPosition((int) SetPos);
                motor_ELBOW.setPower(0.5);
                motor_ELBOW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else {
            while (motor_ELBOW.getCurrentPosition() > SetPos) {
                motor_ELBOW.setTargetPosition((int) SetPos);
                motor_ELBOW.setPower(0.5);
                motor_ELBOW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

    }

    void ResetDrEncoders() {
        motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

    public double GetAvgDrDist() {
        // Average distance travelled by all the wheels
        return (-motor_FLM.getCurrentPosition() + motor_FRM.getCurrentPosition() - motor_RLM.getCurrentPosition() + motor_RRM.getCurrentPosition())/INCH2ENC;
    }

    public double GetAvgStDist() {
//            telemetry.addData("Enc FLM", "%7d", motor_FLM.getCurrentPosition());
//            telemetry.addData("Enc FRM", "%7d", motor_FRM.getCurrentPosition());
//            telemetry.addData("Enc RLM", "%7d", motor_RLM.getCurrentPosition());
//            telemetry.addData("Enc RRM", "%7d", motor_RRM.getCurrentPosition());
//            telemetry.update();
        // Average distance travelled by all the wheels
        return (motor_FLM.getCurrentPosition() + motor_FRM.getCurrentPosition() - motor_RLM.getCurrentPosition() - motor_RRM.getCurrentPosition())/INCH2ENC_ST;
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Cam1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        //Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
//        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("- Size", "%.0f x %.0f", recognition, recognition.getHeight());
            telemetry.update();

            if ("Red Crown" == recognition.getLabel()) {
                if (x>300) {
                    Pixel_Pos = 1;
                }
                else if (x>100) {
                    Pixel_Pos = 2;
                }
            }
        }   // end for() loop
        telemetry.update();
    }   // end method telemetryTfod()
    private void R_grab_open() {
        R_grab.setPosition(0.7);
    }
    private void R_grab_close() {
        R_grab.setPosition(1);
    }
    private void L_grab_open() {
        L_grab.setPosition(0.55);
    }
    private void L_grab_close() {
        L_grab.setPosition(0.3);
    }

}