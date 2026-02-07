package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.DriveConfig;
import org.firstinspires.ftc.teamcode.configs.MentorDriveConfig;
import org.firstinspires.ftc.teamcode.configs.MentorLiftConfig;
import org.firstinspires.ftc.teamcode.configs.MentorRRConfig;
import org.firstinspires.ftc.teamcode.configs.MentorShooterConfig;
import org.firstinspires.ftc.teamcode.configs.RRConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

public class MentorBot extends Robot {
	public enum CarouselPos {
		LEFT,
		CENTER,
		RIGHT;

		//Define functions to make cycling through positions easier
		public CarouselPos next() {
			if (ordinal() == values().length - 1)
				return values()[0];

			return values()[ordinal() + 1];
		}

		public CarouselPos prev() {
			if (ordinal() == 0)
				return values()[values().length - 1];

			return values()[ordinal() - 1];
		}
	}

	public enum CarouselDeliverPos {
		INTAKE,
		CAROUSEL,
		SHOOTER
	}

	private CarouselPos curCarouselPos;
	private CarouselDeliverPos curCarouselDeliverPos;

	private DcMotor frontLift;
	private DcMotor rearLift;

	private CRServo intake;
	private Servo shooterDeliver;
	private Servo carouselDeliver;
	private Servo carousel;

	//Boolean to keep track of our threads
	public volatile boolean intakeBusy = false;

	public DriveConfig driveConfig = new MentorDriveConfig();
	public ShooterConfig shooterConfig = new MentorShooterConfig();
	public RRConfig rrConfig = new MentorRRConfig();
	public MentorLiftConfig liftConfig = new MentorLiftConfig();

	public MentorBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem);

		super.driveConfig = driveConfig;
		super.shooterConfig = shooterConfig;
		super.rrConfig = rrConfig;

		initDrive();
		initShooter();
		initLift();
	}

	public void initShooter() {
		//Initialize shooter motor
		super.initShooter();

		//Initialize intake system
		shooterDeliver = hardwareMap.get(Servo.class, MentorShooterConfig.shooterDeliverString);
		shooterDeliver.setDirection(Servo.Direction.REVERSE);

		intake = hardwareMap.get(CRServo.class, MentorShooterConfig.intakeString);
		intake.setDirection(DcMotorSimple.Direction.REVERSE);

		carouselDeliver = hardwareMap.get(Servo.class, MentorShooterConfig.carouselDeliverString);
		carouselDeliver.setDirection(Servo.Direction.REVERSE);

		carousel = hardwareMap.get(Servo.class, MentorShooterConfig.carouselString);

		moveShooterDeliver(false);
		moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);
		//Allow time for the carousel deliver to get out of the way
		robotSleep(250);
		moveCarousel(CarouselPos.CENTER);
	}

	public void initLift() {
		frontLift = hardwareMap.get(DcMotor.class, MentorLiftConfig.frontLiftString);
		rearLift = hardwareMap.get(DcMotor.class, MentorLiftConfig.rearLiftString);

		frontLift.setDirection(MentorLiftConfig.frontLiftReverse ?
								DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
		rearLift.setDirection(MentorLiftConfig.rearLiftReverse ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

		frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		frontLift.setTargetPosition(0);
		rearLift.setTargetPosition(0);

		frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		frontLift.setPower(MentorLiftConfig.liftSpeed);
		rearLift.setPower(MentorLiftConfig.liftSpeed);
	}

	//Shorthand function to intake into the current carousel selection
	public void autoIntake(Gamepad gamepad) {
		autoIntake(curCarouselPos, gamepad);
	}

	//TODO: Utilize a sensor to determine when a ball has arrived in the carousel deliver
	public void autoIntake(CarouselPos pos, Gamepad gamepad) {
		Thread thread = new Thread() {
			@Override
			public void run() {
				intakeBusy = true;

				//Rotate the carousel to select the right position
				//Check if we can save time
				if (pos != curCarouselPos) {
					//Ensure the carousel deliver won't get in the way
					//Check if we can save time
					if (curCarouselDeliverPos != CarouselDeliverPos.CAROUSEL) {
						moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);

						robotSleep(250);
					}

					moveCarousel(pos);

					//TODO: Add extra time if we are going from LEFT to RIGHT or vice-versa
					// We need 500 ms for the worst case
					// Maybe even use different values for all the different cases
					robotSleep(500);
				}

				//Intake the ball
				moveCarouselDeliver(CarouselDeliverPos.INTAKE);
				//Give the carousel deliver just a bit of time to come down first
				robotSleep(250);
				spinIntake(1);

				//Wait for the user to let go of the button
				while (gamepad.dpad_up) {
					robotSleep(1);
				}

				//Reset the carousel deliver position
				moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);

				//Assist the carousel deliver with the ball before we stop the intake
				robotSleep(250);

				spinIntake(0);

				intakeBusy = false;
			}
		};

		//TODO: Is this actually working the way we want it to?
		// Try checking inside the thread if we are busy
		//Make sure we don't create duplicate threads trying to do the same thing
		if (!intakeBusy)
			thread.start();
	}

	//Shorthand function to dispense the current carousel selection
	public void autoDispense(Gamepad gamepad) {
		autoDispense(curCarouselPos, gamepad);
	}

	public void autoDispense(CarouselPos pos, Gamepad gamepad) {
		Thread thread = new Thread() {
			@Override
			public void run() {
				intakeBusy = true;

				//Rotate the carousel to select the right position
				//Check if we can save time
				if (pos != curCarouselPos) {
					//Ensure the carousel deliver won't get in the way
					//Check if we can save time
					if (curCarouselDeliverPos != CarouselDeliverPos.CAROUSEL) {
						moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);

						robotSleep(250);
					}

					moveCarousel(pos);

					//TODO: Add extra time if we are going from LEFT to RIGHT or vice-versa
					// We need 500 ms for the worst case
					// Maybe even use different values for all the different cases
					robotSleep(500);
				}

				//Dispense the ball
				moveCarouselDeliver(CarouselDeliverPos.INTAKE);
				spinIntake(-1);

				//Wait for the user to let go of the button
				while (gamepad.dpad_down) {
					robotSleep(1);
				}

				//Reset the carousel deliver position
				moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);
				spinIntake(0);

				intakeBusy = false;
			}
		};

		//Make sure we don't create duplicate threads trying to do the same thing
		if (!intakeBusy)
			thread.start();
	}

	public void spinIntake(double speed) {
		//Verify that the carousel is in the proper position
		//Skip verification if we are dispensing
		if (curCarouselDeliverPos == CarouselDeliverPos.INTAKE || speed < 0)
			intake.setPower(speed);
		else
			intake.setPower(0);
	}

	public void cycleCarousel(boolean left, boolean right) {
		if (left)
			moveCarousel(curCarouselPos.prev());
		else if (right)
			moveCarousel(curCarouselPos.next());
	}

	public void moveCarousel(CarouselPos pos) {
		if (curCarouselDeliverPos == CarouselDeliverPos.CAROUSEL) {
			carousel.setPosition(MentorShooterConfig.carouselPositions[pos.ordinal()]);
			curCarouselPos = pos;
		}
	}

	public void moveCarouselDeliver(CarouselDeliverPos pos) {
		carouselDeliver.setPosition(MentorShooterConfig.carouselDeliverPositions[pos.ordinal()]);
		curCarouselDeliverPos = pos;
	}

	public void moveShooterDeliver(boolean pos) {
		shooterDeliver.setPosition(pos ? 1 : 0);
	}

	public void moveLift(boolean up, boolean down) {
		if (up) {
			frontLift.setTargetPosition(MentorLiftConfig.liftMax);
			rearLift.setTargetPosition(MentorLiftConfig.liftMax);
		} else if (down) {
			frontLift.setTargetPosition(0);
			rearLift.setTargetPosition(0);
		}

		telemetry.addData("Front Lift Target", frontLift.getTargetPosition());
		telemetry.addData("Front Lift Pos", frontLift.getCurrentPosition());
		telemetry.addData("Rear Lift Target", rearLift.getTargetPosition());
		telemetry.addData("Rear Lift Pos", rearLift.getCurrentPosition());
	}
}
