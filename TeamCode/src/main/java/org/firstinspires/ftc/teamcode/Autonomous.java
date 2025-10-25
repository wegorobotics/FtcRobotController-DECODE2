//test
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name= "Autonomous")

public class Autonomous extends LinearOpMode {
    // motors
    DcMotor fl_Wheel;
    DcMotor bl_Wheel;
    DcMotor fr_Wheel;
    DcMotor br_Wheel;
    DcMotor placing_slide;
    DcMotor climbing_slide;
    //private Limelight3A limelight;
    Servo arm_servo;
    Servo claw_servo;

    @Override
    public void runOpMode(){
        fl_Wheel = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_Wheel = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_Wheel = hardwareMap.get(DcMotor.class, "fr_motor");
        br_Wheel = hardwareMap.get(DcMotor.class, "br_motor");
        placing_slide = hardwareMap.get(DcMotor.class, "placing_motor");
        climbing_slide = hardwareMap.get(DcMotor.class, "climbing_motor");
        //limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        arm_servo = hardwareMap.get(Servo.class, "arm");
        claw_servo = hardwareMap.get(Servo.class, "claw");

        fr_Wheel.setDirection(DcMotor.Direction.REVERSE);
        fl_Wheel.setDirection(DcMotor.Direction.FORWARD);
        br_Wheel.setDirection(DcMotor.Direction.REVERSE);
        bl_Wheel.setDirection(DcMotor.Direction.REVERSE);

        fr_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        placing_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbing_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //limelight.start();

        waitForStart();
        arm_servo.setPosition(0.17);
        claw_servo.setPosition(0.6);

        //placing_slide.setPower(-0.8);
        //climbing_slide.setPower(0.8);
        move(1, 90, 0, 2500);
        move(1, 270, 0, 100);
        placing_slide.setPower(-0.6);
        climbing_slide.setPower(0.6);
        sleep(1500);
        placing_slide.setPower(0);
        climbing_slide.setPower(-0);
        claw_servo.setPosition(0.42);
        arm_servo.setPosition(0.25);
        move(1, 270, 0, 300);
        placing_slide.setPower(0.4);
        climbing_slide.setPower(-0.4);
        sleep(5000);
        placing_slide.setPower(0);
        climbing_slide.setPower(-0);
        claw_servo.setPosition(0.42);
        sleep(400);
    }

    public void move(double magnitude, double direction, double turn, long time) {
        double radians = 1 * ((direction) / 180) * Math.PI;

        fr_Wheel.setPower(1.024 * (-1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);
        br_Wheel.setPower(1.000 * (1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude - turn) / 2);
        fl_Wheel.setPower(1.094 * (-1 * Math.sin(radians + (0.25 * Math.PI)) * magnitude - turn) / 2);
        bl_Wheel.setPower(0.989 *  (1 * Math.sin(radians - (0.25 * Math.PI)) * magnitude + turn) / 2);

        sleep(time);

        fr_Wheel.setPower(0);
        bl_Wheel.setPower(0);
        fl_Wheel.setPower(0);
        br_Wheel.setPower(0);
    }
}