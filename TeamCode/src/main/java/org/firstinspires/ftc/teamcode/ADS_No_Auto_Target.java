package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.CompDriveConfig;
import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.PIDController;

import java.util.List;


@Autonomous(name="ADS", group="Linear OpMode")
public class ADS extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    CompBot robot;
    Limelight3A limelight;
    PIDController pid = new PIDController(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CompBot(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();


        Pose2d Drive_from_start = new Pose2d(-62.64, -17.76, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);
        double prevCameraTime = runtime.milliseconds();
        double curCameraTime;
        boolean foundTarget;
        double cameraAngle = 0;
        double cameraDistance = 0;
        boolean spinShooter = false;
        pid.setSetpoint(0);
        pid.setOutputLimits(-1, 1);
        pid.setPID(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
        pid.setPID(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
        curCameraTime = runtime.milliseconds();
        foundTarget = false;
        robot.moveTrigger(false);

        //TODO: Reject positioning if we see obelisk tags
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            //For use with sensor fusion
//				Pose3D pose = result.getBotpose_MT2();

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                cameraDistance = fiducial.getRobotPoseTargetSpace().getPosition().z;
                cameraAngle = fiducial.getTargetXDegrees();

                foundTarget = true;
                telemetry.addData("Target Found","yes");
                telemetry.addData("Fiducial " + id, -cameraDistance * 39.37007874015748031496  + " inches away");
                telemetry.addData("Fiducial " + id, cameraAngle + " degrees");
            }
        }else telemetry.addData("TARGET NOT FOUND", "REINIT");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, cameraDistance);
        sleep(2000);
        robot.moveGuard(true);
        sleep(150);
        robot.moveTrigger(true);
        sleep(20000);





    }
}


