package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.PIDController;


@Autonomous(name="ADS_Far_Red", group="Linear OpMode")
public class ADS_Far_Red extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    CompBot robot;
    Camera camera;

    PIDController pid = new PIDController(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CompBot(hardwareMap, telemetry);
        camera = new Camera(hardwareMap, telemetry, runtime);
        camera.targetID = camera.RED_ID;

        Pose2d Drive_from_start = new Pose2d(62, 6, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);

        boolean spinShooter = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(Drive_from_start)
                        .strafeToLinearHeading(new Vector2d(58,6),Math.toRadians(-25))
                        .build());

//        for (int i = 0; i < 1000; i++) {
//            camera.update();
//            telemetry.update();
//        }

//        robot.fireShooterDistance(true, camera.getCameraDistance());
        robot.fireShooterDistance(true, 130);
        sleep(2500);
        robot.moveGuard(true);
        sleep(500);
        for (int i = 0; i < 3; i++) {
            robot.moveTrigger(true);
            sleep(500);
            robot.moveTrigger(false);
            sleep(500);
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(500);
        }
        sleep(500);
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooterDistance(false,0);
        Pose2d LP_P1 = new Pose2d(new Vector2d(58,6),Math.toRadians(-25));

        Actions.runBlocking(


                drive.actionBuilder(LP_P1)
                        .strafeToLinearHeading(new Vector2d(58,50),Math.toRadians(0))
                        .build());
        /*
        robot.spinIntake(0);
        Pose2d P1_LP = new Pose2d(new Vector2d(36,-48),Math.toRadians(270));
        spinShooter=true;
        robot.fireShooter(spinShooter,2);
        Actions.runBlocking(
                drive.actionBuilder(P1_LP)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, -6, Math.toRadians(25)), Math.toRadians(25))
                        .build()
        );
        for (int i = 0; i < 500; i++) {
            camera.update();
            telemetry.update();
        }
        robot.fireShooterDistance(spinShooter, camera.getCameraDistance());
        sleep(2500);
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 3; i++) {
            robot.moveTrigger(true);
            sleep(500);
            robot.moveTrigger(false);
            sleep(500);
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(500);
        }
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooter(spinShooter,2);
        Pose2d LP_P2 = new Pose2d(new Vector2d(58,-6),Math.toRadians(25));
        Actions.runBlocking(
                drive.actionBuilder(LP_P2)
                        .strafeToLinearHeading(new Vector2d(49,-6),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(49,-65),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(49,-35),Math.toRadians(25))
                        .build()

        );
        robot.spinIntake(0);
        Pose2d P2_LP = new Pose2d(new Vector2d(49,-35),Math.toRadians(270));
        spinShooter=true;
        robot.fireShooter(spinShooter,2);
        Actions.runBlocking(
                drive.actionBuilder(P2_LP)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, -6, Math.toRadians(25)), Math.toRadians(25))
                        .build()
        );
        for (int i = 0; i < 500; i++) {
            camera.update();
            telemetry.update();
        }
        robot.fireShooterDistance(spinShooter, camera.getCameraDistance());
        sleep(2500);
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(700);
        }
        sleep(400);
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooter(spinShooter,1);

        Pose2d LP_EP = new Pose2d(new Vector2d(56,-8),Math.toRadians(25));
        Actions.runBlocking(
                drive.actionBuilder(LP_EP)
                        .strafeToLinearHeading(new Vector2d(50,-6),Math.toRadians(25 ))
                        .build()


        );


         */


    }
}


