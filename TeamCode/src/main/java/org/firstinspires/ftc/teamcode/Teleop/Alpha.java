package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "АЛЬФА")
public class Alpha extends LinearOpMode {

    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C

    // Drive power values
    private DcMotorEx Nectar; // 0E
    private DcMotorEx Pollen; // 1E
    private DcMotor LS; //2E
    private DcMotorEx INTAKE; // 3E
    private CRServo FLYWHEELGATE; //5S
    private CRServo FLOWERGATE; // 2S
    private CRServo UNLOCK; //4S
    private CRServo CamServo; //3S
    // PID-controlled linear slide (see LinearSlideController.java)
    private LinearSlideController slide;
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    private boolean shootingNectar = false;
    private boolean shootingPollen = false;
    private boolean FLOWER_STATE = false;
    private boolean FLYWHEEL_STATE = false;
    private boolean DRIVE_STATE = false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double NECTAR_VEL = 100000;
    public static double POLLEN_VEL = 100000;
    public static double INTAKE_VEL = 7500;

    //flower once
    public static long flowerNectarOnceTime = 400;
    public static long flowerPollenOnceTime = 350;

    //flywheel gate
    public static double posUp = 0.35;
    public static double posDown = -0.95;

    //
    private double flowerClose = 0;
    private double flowerOpen = 0.75;

    //flow gate
    private double stopFlow = -0.4;
    private double letFlow = 0;
    public static long flowtime = 400;

    //unlock flywheels
    private double unlockpos = 0.15;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Nectar = hardwareMap.get(DcMotorEx.class, "NECTAR");
        Pollen = hardwareMap.get(DcMotorEx.class, "POLLEN");
        LS = hardwareMap.get(DcMotor.class, "LS");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        FLYWHEELGATE = hardwareMap.get(CRServo.class, "FLYWHEEL");
        FLOWERGATE = hardwareMap.get(CRServo.class, "FLOWER");
        UNLOCK = hardwareMap.get(CRServo.class, "UNLOCK");


        // Directions
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        Pollen.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);
        INTAKE.setDirection(DcMotorEx.Direction.REVERSE);
        FLOWERGATE.setDirection(CRServo.Direction.REVERSE);
        FLYWHEELGATE.setPower(posUp);
        FLOWERGATE.setPower(flowerClose);
        UNLOCK.setPower(unlockpos);


        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Linear slide (slide must be fully down at init)
        slide = new LinearSlideController(LS);
        slide.init();

        telemetry.addLine("Привет!");
        //telemetry.addLine("안녕하세요");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        slide.start();

        while (opModeIsActive()) {
            Drive();
            Launcher();
            Intake();
            LinearSlide();
            FlywheelGate();
            FlowerGate();
            FlowerNectarOnce();
            FlowerPollenOnce();
            Flow();


            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Drive State: ", DRIVE_STATE);
            telemetry.addData("Nectar Target:", NECTAR_VEL);
            telemetry.addData("Nectar Actual: ", Nectar.getVelocity());
            telemetry.addData("Pollen:", POLLEN_VEL);
            telemetry.addData("Pollen Actual: ", Pollen.getVelocity());
            /*telemetry.addData("Slide Target", slide.getTarget());
            telemetry.addData("Slide Pos", slide.getPosition());
            telemetry.addData("Slide Power", slide.getPower());*/
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            /*telemetry.addData("Flywheel Power: ", FLYWHEELGATE.getPower());
            telemetry.addData("Flower Power: ", FLOWERGATE.getPower());*/

            telemetry.update();
        }
    }

    // Shooter control
    private void Launcher() {

        if (gamepad1.b){
            Nectar.setVelocity(NECTAR_VEL);
        }
        if (gamepad1.a){
            Pollen.setVelocity(POLLEN_VEL);
        }
        if (gamepad1.x){
            Nectar.setVelocity(0);
            Pollen.setVelocity(0);
            shootingNectar = false;
            shootingPollen = false;
        }
    }


    private void Intake() {
        if (gamepad1.left_trigger>0)
            INTAKE.setVelocity(INTAKE_VEL);
        else if (gamepad1.left_bumper)
            INTAKE.setVelocity(-INTAKE_VEL);
        else
            INTAKE.setVelocity(0);
    }

    // Linear slide: dpad_up toggles between up/down positions, held by PID
    private void LinearSlide() {
        slide.update(gamepad1.dpad_up);
    }

    public void Flow() {
        if (gamepad1.right_trigger>0) {
            FLYWHEELGATE.setPower(posDown);
            sleep(flowtime);
            FLYWHEELGATE.setPower(posUp);
            sleep(flowtime);
        }
    }

    public void FlywheelGate() {
        if (gamepad1.y) {
            FLYWHEEL_STATE=!FLYWHEEL_STATE;
            if(FLYWHEEL_STATE) FLYWHEELGATE.setPower(posDown);
            else FLYWHEELGATE.setPower(posUp);
            sleep(200);
        }
    }

    public void FlowerGate() {
        if (gamepad1.dpad_left) {
            FLOWER_STATE=!FLOWER_STATE;
            if(FLOWER_STATE) FLOWERGATE.setPower(flowerOpen);
            else FLOWERGATE.setPower(flowerClose);
            sleep(200);
        }
    }

    public void FlowerNectarOnce() {
        if (gamepad1.dpad_down) {
            FLOWERGATE.setPower(flowerClose);
            sleep( 10);
            FLOWERGATE.setPower(flowerOpen);
            sleep( flowerNectarOnceTime);
            FLOWERGATE.setPower(flowerClose);
        }
    }

    public void FlowerPollenOnce() {
        if (gamepad1.dpad_right) {
            FLOWERGATE.setPower(flowerClose);
            sleep( 10);
            FLOWERGATE.setPower(flowerOpen);
            sleep( flowerPollenOnceTime);
            FLOWERGATE.setPower(flowerClose);
        }
    }

    private void Drive(){
        if (gamepad1.right_bumper) DRIVE_STATE = !DRIVE_STATE;
        if (DRIVE_STATE) MecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x );
        else MecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x,gamepad1.right_stick_x );
    }
    private void MecanumDrive(double forward, double strafe, double turn){
        // Mecanum drive calculations

        lbPower = forward - strafe + turn;
        lfPower = forward + strafe + turn;
        rbPower = forward + strafe - turn;
        rfPower = forward - strafe - turn;

        // Normalize
        double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower)));

        if (max > 1.0) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        LB.setPower(lbPower);
        LF.setPower(lfPower);
        RB.setPower(rbPower);
        RF.setPower(rfPower);
    }
}