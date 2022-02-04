package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriverControlBlue extends LinearOpMode {
    public DcMotor fl; //front left
    public DcMotor fr; //front right
    public DcMotor bl; //back left
    public DcMotor br; //back right
    public DcMotor ca; //carousel thing
    public DcMotor cs; //claw spinner
    //public Servo ac; //arm claw
    public DcMotor ls; //linear slide mover
    //public CRServo ds; //drawer slide mover
    //public CRServo cf; //claw flipper

//Above declares the motors/servos for the code

    @Override
    public void runOpMode() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        ca = hardwareMap.dcMotor.get("ca");
        //ac = hardwareMap.servo.get("ac");
        ls = hardwareMap.dcMotor.get("ls");
        cs = hardwareMap.dcMotor.get("cs");
        //ds = hardwareMap.crservo.get("ds");
        //cf = hardwareMap.crservo.get("cf");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
//Adding above into code.  When code is active, this is active.  Defines the wheels and wheel direction

        cs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode((DcMotor.RunMode.RUN_USING_ENCODER));

        double m = 1;
        int startPosition = ls.getCurrentPosition();
        //int startPosition = -480;
        System.out.print(startPosition);

        /*
        //this is the first one we made but it does nto trun correctly
        //driving with the forward and turn on the right stick
        waitForStart();
        while (opModeIsActive()) {
            double drive = gamepad1.right_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            bl.setPower(m * (-gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
            br.setPower(m * (-gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
            fr.setPower(m * (gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            fl.setPower(m * (-gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
        */

        //below used for testing turing
        //this one should work now and should be the one that we use from now on
        waitForStart();
        while (opModeIsActive()) {
            double drive = gamepad1.right_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            bl.setPower(m * (-gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
            br.setPower(m * (-gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
            fr.setPower(m * (gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            fl.setPower(m * (-gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));

            //telemetry.addData("Start Position", startPosition);
            //the != can be after the gamepad.whatever thing and then add the opposite button to make it so if both of the buttons are pressed
            //the motor does not torque itself trying to activate both functions


            //ARM CODE
            //below allows the driver to limit the max power output to the drive wheels
            if (gamepad1.x) {
                m = (0.5);
            }

            if (gamepad1.y) {
                m = (1);

            }


            //example of a one button system for activating a piece of the robot
            //this does not work very well and is not recommended by NYAN
            /*
            if (gamepad1.a) {
                if (m < .75) {
                    m = 1;
                } else {
                    m = .25;
                }
            */


            //carousel mover thing below
            if (gamepad2.a) {
                ca.setPower(0.5);
                sleep(1000);
                ca.setPower(0.75);
            }

            if (gamepad2.b) {
                ca.setPower(0);
            }



        //linear slide positions style
        if (gamepad2.dpad_up) {
            ls.setTargetPosition(startPosition + 2000);
            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls.setPower(1);

        }

        if (gamepad2.dpad_left) {
            ls.setTargetPosition(startPosition + 1300);
            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls.setPower(1);

        }

        if (gamepad2.dpad_right) {
            ls.setTargetPosition(startPosition + 850);
            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls.setPower(1);

        }

        if (gamepad2.dpad_down) {
            ls.setTargetPosition(startPosition);
            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls.setPower(-0.5);

        }
            telemetry.addData("encoder-ls" , ls.getCurrentPosition()+" busy=" +ls.isBusy());
            telemetry.update();


        /*
        //linear slide manual control
        if (gamepad2. x) {
            ls.setPower(1);
        }

        if (gamepad2.y) {
            ls.setPower(-1);

        else {
        ls.setPower(0);
        }
        */


        /*
        //claw flipper thing
        if (gamepad2.x) {
            cf.setPower(1);
        }
        */



        //"claw" stuff
        if (gamepad2.right_bumper) {
            cs.setPower(-1);
        }

        if (gamepad2.left_bumper) {
            cs.setPower(1);
        }

        else {
            cs.setPower(0);
        }




        }

    }

}
