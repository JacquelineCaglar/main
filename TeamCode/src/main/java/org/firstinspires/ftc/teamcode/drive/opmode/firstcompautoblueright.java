package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class firstcompautoblueright extends LinearOpMode {
    public DcMotor fl; //front left
    public DcMotor fr; //front right
    public DcMotor bl; //back left
    public DcMotor br; //back right
    public DcMotor ca; //carousel thing

    @Override
    public void runOpMode() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        ca = hardwareMap.dcMotor.get("ca");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        fr.setPower(0.5);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        br.setPower(-0.5);
        sleep(100);
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        sleep(1000);
        fr.setPower(1);
        fl.setPower(1);
        bl.setPower(-1);
        br.setPower(1);
        sleep(700);
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fr.setPower(-0.5);
        fl.setPower(0.5);
        bl.setPower(0.5);
        br.setPower(0.5);
        sleep(100);
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        ca.setPower(0.5);
        sleep(5000);

        fr.setPower(0.5);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        br.setPower(-0.5);
        sleep(1000);
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        sleep(880);
        fr.setPower(1);
        fl.setPower(1);
        bl.setPower(-1);
        br.setPower(1);
        sleep(400);
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);


    }
}