package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="ADS_RED_FRONT", group="Linear OpMode")
public class ADS_RED_FRONT extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake = null;
    private DcMotor Shooter = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private Servo Trigger = null;
    public static double shooter_pre_A = .8;
    public static double shooter_pre_B = .75;
    public static double shooter_pre_C = .7;
    double shooter_pre_slecter = 0;
    public static double Start_R1_X=0 ;
    public static double Start_R1_Y= 0;
    public static double Start_R1_H= 0;
    @Override
    public void runOpMode() {

    }
}


