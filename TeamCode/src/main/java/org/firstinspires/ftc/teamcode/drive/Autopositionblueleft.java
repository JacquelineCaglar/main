package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.WebCamExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class Autopositionblueleft extends LinearOpMode {


    public DcMotor fl; //front left
    public DcMotor fr; //front right
    public DcMotor bl; //back left
    public DcMotor br; //back right
    public DcMotor ca; //carousel thing
    public CRServo cs; //claw spinner
    //public Servo ac; //arm claw
    public DcMotor ls; //linear slide mover
    //public CRServo ds; //drawer slide mover
    //public CRServo cf; //claw flipper

    OpenCvCamera webcam;
    WebCamExample.DeliveryLocationDeterminationPipeline pipeline;
    int location=0;
    @Override
    public void runOpMode(){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        // ca = hardwareMap.dcMotor.get("ca");
        //ac = hardwareMap.servo.get("ac");
        ls = hardwareMap.dcMotor.get("ls");
        cs = hardwareMap.crservo.get("cs");
        //ds = hardwareMap.crservo.get("ds");
        //cf = hardwareMap.crservo.get("cf");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new WebCamExample.DeliveryLocationDeterminationPipeline();
        webcam.setPipeline(pipeline);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); //keep it at 480p
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("camera failed to open");
                telemetry.update();
            }
        });
        while (!opModeIsActive()) {
            telemetry.addData("Object Position", WebCamExample.deliveryLocation);
            telemetry.update();
            location=WebCamExample.deliveryLocation;
        }

        waitForStart();
        webcam.stopStreaming();
        if(location == -1) {
            leftCode();
        }
        else if(location == 0){
            centerCode();
        }
        else{
            rightCode();

        }

    }

    public void leftCode(){
        //when left spot is taken
        //To import, press Alt and enter
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-65,40 ); //might have to change coordinates
        drive.setPoseEstimate(startPose);
        Trajectory firstOne0 = drive.trajectoryBuilder(startPose)
            .forward(65)
            .build();
        Trajectory secondOne0 = drive.trajectoryBuilder(firstOne0.end())
                .strafeRight(15)
                .build();
        Trajectory thirdOne0 = drive.trajectoryBuilder(secondOne0.end())
                .strafeLeft(20)
                .build();

        //MOTION STARTS HERE
        drive.followTrajectory(firstOne0);
        ls.setPower(.50);
        sleep(3000); //only need to change this for new paths
        ls.setPower(0);
        drive.followTrajectory(secondOne0);
        sleep(500);
        cs.setPower(1);
        sleep(5000);
        cs.setPower(0);
        sleep(1000);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(thirdOne0);




    }
    public void centerCode(){
        //when center spot is taken
        //To import, press Alt and enter
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-65,40 ); //might have to change coordinates
        drive.setPoseEstimate(startPose);
        Trajectory firstOne0 = drive.trajectoryBuilder(startPose)
                .forward(65)
                .build();
        Trajectory secondOne0 = drive.trajectoryBuilder(firstOne0.end())
                .strafeRight(15)
                .build();
        Trajectory thirdOne0 = drive.trajectoryBuilder(secondOne0.end())
                .strafeLeft(20)
                .build();

        //MOTION STARTS HERE
        drive.followTrajectory(firstOne0);
        ls.setPower(.50);
        sleep(3100); //only need to change this for new path
        ls.setPower(0);
        drive.followTrajectory(secondOne0);
        sleep(500);
        cs.setPower(1);
        sleep(5000);
        cs.setPower(0);
        sleep(1000);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(thirdOne0);
    }
    public void rightCode(){
        //when right spot is taken
        //To import, press Alt and enter
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-65,40 ); //might have to change coordinates
        drive.setPoseEstimate(startPose);
        Trajectory firstOne0 = drive.trajectoryBuilder(startPose)
                .forward(65)
                .build();
        Trajectory secondOne0 = drive.trajectoryBuilder(firstOne0.end())
                .strafeRight(15)
                .build();
        Trajectory thirdOne0 = drive.trajectoryBuilder(secondOne0.end())
                .strafeLeft(20)
                .build();

        //MOTION STARTS HERE
        drive.followTrajectory(firstOne0);
        ls.setPower(.50);
        sleep(3200); //only need to change this for new paths
        ls.setPower(0);
        drive.followTrajectory(secondOne0);
        sleep(500);
        cs.setPower(1);
        sleep(5000);
        cs.setPower(0);
        sleep(1000);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(thirdOne0);
    }


}