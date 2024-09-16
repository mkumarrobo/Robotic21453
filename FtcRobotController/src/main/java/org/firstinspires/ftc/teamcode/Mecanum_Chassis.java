package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Super Smash Bros Ultimate", group = "Iterative Opmode")
public class Mecanum_Chassis extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor motor_FRM = null;
    private DcMotor motor_FLM = null;
    private DcMotor motor_RLM = null;
    private DcMotor motor_RRM = null;
    private DcMotor motor_ELBOW = null;
    private DcMotor motor_XTND = null;
    private DcMotor motor_RISR = null;
    private Servo L_grab = null;
    private Servo R_grab = null;
    private Servo Airplane = null;
    DistanceSensor Dist_OR;
    DistanceSensor Dist_OL;
    DistanceSensor Dist_IR;
    DistanceSensor Dist_IL;
//    Translation2d

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        // Initialize DC Motors
        motor_FLM = hardwareMap.get(DcMotor.class, "FLM");
        motor_FRM = hardwareMap.get(DcMotor.class, "FRM");
        motor_RLM = hardwareMap.get(DcMotor.class, "RLM");
        motor_RRM = hardwareMap.get(DcMotor.class, "RRM");
        motor_ELBOW = hardwareMap.get(DcMotor.class, "ELBOW");
        motor_XTND = hardwareMap.get(DcMotor.class, "XTND");
        motor_RISR = hardwareMap.get(DcMotor.class, "RISR");

        // Initialize Servo Motors
        L_grab = hardwareMap.get(Servo.class, "Grapple_Left");
        R_grab = hardwareMap.get(Servo.class, "Grapple_Right");
        Airplane = hardwareMap.get(Servo.class, "AIRPLANE");

        // Initialize Sensors
        Dist_OR = hardwareMap.get(DistanceSensor.class, "DIST_OR");
        Dist_OL = hardwareMap.get(DistanceSensor.class, "DIST_OL");
        Dist_IR = hardwareMap.get(DistanceSensor.class, "DIST_IR");
        Dist_IL = hardwareMap.get(DistanceSensor.class, "DIST_IL");

        // Setup lifting DC motors for drive by encoder
        motor_ELBOW.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor_XTND.setDirection(DcMotorSimple.Direction.REVERSE);

//        motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_ELBOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_XTND.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_ELBOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_XTND.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive = 0;
        double strafe = 0;
        double twist = 0;
        double desired_distance = 20;
        double SPIKE = 0.5;
        double Air = (Dist_IR.getDistance(DistanceUnit.CM) + Dist_IL.getDistance(DistanceUnit.CM))/2;
        double strafedelta = 0;
        double twistdelta = 0;
        double ELBOW_SetPos;
        double XTND_SetPos;

        ELBOW_SetPos = gamepad2.left_stick_y * 1000;
        XTND_SetPos = -gamepad2.right_stick_y * 850 + 5;

        if (gamepad2.a) {
            XTND_SetPos = 40000/Air;
        }


        if (gamepad1.right_bumper) {
            SPIKE = 0.2;
        }
       if (gamepad1.x && (Dist_OL.getDistance(DistanceUnit.CM)<50 || Dist_OR.getDistance(DistanceUnit.CM)<50 || Dist_IL.getDistance(DistanceUnit.CM)<50 || Dist_IR.getDistance(DistanceUnit.CM)<50)) {
           strafedelta = 5*(1.5/Dist_OR.getDistance(DistanceUnit.CM) + 1/Dist_IR.getDistance(DistanceUnit.CM) - (1.5/Dist_OL.getDistance(DistanceUnit.CM) + 1/Dist_IL.getDistance(DistanceUnit.CM)));
           if (strafedelta > 0.5) {
               strafedelta = 0.5;
           }
           else if (strafedelta < -0.5) {
               strafedelta = -0.5;
           }
           twistdelta = strafedelta * 0.0;
        }


        if (gamepad1.a){
            if (Air < 30) {
                drive = -((Air - desired_distance)/100);
            }

            strafe = 0;
            twist = (Dist_IR.getDistance(DistanceUnit.CM) - Dist_IL.getDistance(DistanceUnit.CM))/20;
            if (twist>0.5){
                twist = 0.5;
            }
            else if (twist<-0.5){
                twist = -0.5;
            }

        }

        else {
            drive = -gamepad1.left_stick_y;
//            drive = -gamepad1.right_trigger;
            strafe = -gamepad1.left_stick_x * 1.1 + strafedelta;
            twist = -gamepad1.right_stick_x + twistdelta;
        }

//        telemetry.addData("Left_Trigger", "%.3f", gamepad2.left_trigger);
////        telemetry.addData("Right_Trigger", "%.3f", gamepad2.right_trigger);
//
        telemetry.addData("Distance Out Right", "%.3f", Dist_OR.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Out Left", "%.3f", Dist_OL.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance In Right", "%.3f", Dist_IR.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance In Left", "%.3f", Dist_IL.getDistance(DistanceUnit.CM));

//        telemetry.addData("Strafe", "%.3f", strafe);
        telemetry.addData("Enc XTND", "%7d", motor_XTND.getCurrentPosition());
        telemetry.addData("Enc ELBOW", "%7d", motor_ELBOW.getCurrentPosition());
//        telemetry.addData("Enc FLM", "%7d", motor_FLM.getCurrentPosition());
//        telemetry.addData("Enc FRM", "%7d", motor_FRM.getCurrentPosition());
//        telemetry.addData("Enc RLM", "%7d", motor_RLM.getCurrentPosition());
//        telemetry.addData("Enc RRM", "%7d", motor_RRM.getCurrentPosition());
//        telemetry.addData("Enc AVG", "%7d", motor_FLM.getCurrentPosition() + motor_FRM.getCurrentPosition() + motor_RLM.getCurrentPosition() + motor_RRM.getCurrentPosition());
//        telemetry.addData("Enc XTND CMD", "%7d", (int) XTND_SetPos);
//        telemetry.addData("Enc ELBOW CMD", "%.3f", ELBOW_SetPos);
//        telemetry.addData("Enc ELBOW", "%7d", telemetry.addData("Enc ELBOW", "%7d", -gamepad2.left_stick_y ));
        //        telemetry.addData("Enc ELBOW", "%7d", telemetry.addData("Enc ELBOW", "%7d", -gamepad2.left_stick_y ));
        telemetry.update();


        /*
          * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

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
        if(gamepad1.left_bumper){
            SPIKE= 1;
        }


        if (gamepad2.b && gamepad2.y) {
            Airplane.setPosition(0.75);
        }
        else {
            Airplane.setPosition(1);
        }

        if (gamepad2.left_bumper) {
            L_grab.setPosition(0.55);
        }
        else {
            L_grab.setPosition(0.3);
        }

        if(gamepad2.right_bumper) {
            R_grab.setPosition(0.7);
        }

        else {
            R_grab.setPosition(1);
        }

        motor_FLM.setPower(speeds[0] * SPIKE);
        motor_FRM.setPower(-speeds[1] * SPIKE);
        motor_RLM.setPower(speeds[2] * SPIKE);
        motor_RRM.setPower(-speeds[3] * SPIKE);

        motor_ELBOW.setTargetPosition((int) ELBOW_SetPos);
        motor_XTND.setTargetPosition((int) XTND_SetPos);

        motor_ELBOW.setPower(0.5);
        motor_XTND.setPower(1);

        if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            motor_ELBOW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        motor_XTND.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad2.dpad_up){
            motor_RISR.setPower(1);
        }
        else if (gamepad2.dpad_down){
            motor_RISR.setPower(-1);
        }
        else {
            motor_RISR.setPower(0);
        }


        //L_grab.setPosition(gamepad2.left_trigger);
        //R_grab.setPosition(gamepad2.right_trigger);

    }
}