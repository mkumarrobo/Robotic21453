//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Autonomous(name = "Russian Tank Drive")
//public class Testings extends LinearOpMode {
//
//    private DcMotor motor_FRM = null;
//    private DcMotor motor_FLM = null;
//    private DcMotor motor_RLM = null;
//    private DcMotor motor_RRM = null;
//    private DcMotor motor_ELBOW = null;
//    private DcMotor motor_XTND = null;
//    private DcMotor motor_RISR = null;
//    private Servo L_grab = null;
//    private Servo R_grab = null;
//    DistanceSensor Dist_R;
//    DistanceSensor Dist_L;
//    private double Enc2Inc = 1000/9;
//
//    @Override
//    public void runOpMode() {
//        motor_FLM = hardwareMap.get(DcMotor.class, "FLM");
//        motor_FRM = hardwareMap.get(DcMotor.class, "FRM");
//        motor_RLM = hardwareMap.get(DcMotor.class, "RLM");
//        motor_RRM = hardwareMap.get(DcMotor.class, "RRM");
//        motor_ELBOW = hardwareMap.get(DcMotor.class, "ELBOW");
//        motor_XTND = hardwareMap.get(DcMotor.class, "XTND");
//        motor_RISR = hardwareMap.get(DcMotor.class, "RISR");
//        L_grab = hardwareMap.get(Servo.class, "Grapple_Left");
//        R_grab = hardwareMap.get(Servo.class, "Grapple_Right");
//        Dist_R = hardwareMap.get(DistanceSensor.class, "DIST_R");
//        Dist_L = hardwareMap.get(DistanceSensor.class, "DIST_L");
//
//        double gripperPosition = 0.0;
//
//
//        gripperhit = 0;
//        leftPos = 0;
//        rightPos = 0;
//
//
//            // Mecanum drive is controlled with three axes: drive (front-and-back),
//            // strafe (left-and-right), and twist (rotating the whole chassis).
//            double drive = 0;
//            double strafe = 0;
//            double twist = 0;
//            double desired_distance = 50;
//            double SPIKE = 0.5;
//            double Air = ((Dist_R.getDistance(DistanceUnit.CM) + (Dist_L.getDistance(DistanceUnit.CM))))/2;
//            double strafedelta = 0;
//            double twistdelta = 0;
//
//
//        waitForStart();
//        drive(1000, 1000, 50, 0.25);
//        sleep(1000);
//        drive(900, 900, 50, 0.25);
//        sleep(1000);
//        drive(600, 600, 2000, .25);
//        sleep(1000);
//        drive(-400, 400, 2000, .25);
//        sleep(1000);
//        drive(25, 25, -4000, .25);
//        sleep(5500);
//
//        drive(-300, -300, 0, .25);
//        sleep(1000);
//        drive(1100, -1100, 0, .25);
//        sleep(1000);
//        drive(1600, 1600, 200, .25);
//        sleep(1000);
//
////            drive(-1600, -1600, 200, .25);
////          sleep(1000);
////        drive(-900, 900, 1000, 25);
//        //      sleep(1000);
//        //    drive(300, 300, 2000, .25);
//        //  sleep(1000);
//        //drive(-50, 50, 2000, .25);
//        //sleep(1000);
//        //drive(-1, -1, -5000, .25);
//        //sleep(1111);
//
//
//    }
//
//    private void drive(int leftTarget, int rightTarget, int gripperTarget, double speed) {
//
//
//        leftPos += leftTarget;
//        rightPos += rightTarget;
//        gripperhit += gripperTarget;
//
//        motor_FRM.setTargetPosition(gripperhit);
//        motor_FLM.setTargetPosition(leftPos);
//        motor_RLM.setTargetPosition(leftPos);
//        motor_RRM.setTargetPosition(leftPos);
//        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        up.setPower(speed);
//        left.setPower(speed);
//        right.setPower(speed);
//
//        while (opModeIsActive() && left.isBusy() && right.isBusy() && up.isBusy()) {
//            idle();
//        }
//    }
//
//}