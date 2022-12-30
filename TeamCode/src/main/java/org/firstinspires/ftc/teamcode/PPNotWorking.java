
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pid.kp;
import static org.firstinspires.ftc.teamcode.pid.ki;
import static org.firstinspires.ftc.teamcode.pid.kd;
import static org.firstinspires.ftc.teamcode.pid.kf;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class PPNotWorking extends OpMode{
    //private Gyroscope imu;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor slider,arm;
    private DcMotor slider2,arm2;
    private Servo claw;
    private TouchSensor touchy;
    double sm = 1, ms = 2;
    double poz = 0;
    double gpoz = 0.5;
    private BNO055IMU imu;
    double y, x, rx;
    double sliderPower,lastsliderPower,lastGamepadSlider;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double lastTime;
    private boolean alast = false;
    float right_stick2;
    float right_stick1;
    boolean v = true,ok1,ok2,ok3,ok4,ok5,ok6,ok7;
    boolean FirstTime = true;
    boolean inchis = false;
    boolean overpower = true;
    boolean permisie = true;
    boolean stopDJ = false;
    boolean tru=false;
    boolean touch=false;
    private boolean stop=false;
    int okGrip = 1, okClaw = 1;
    public int i;
    public int cn=0;
    private double correction=0;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25;
    int loaderState = -1;
    private int apoz = 0;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0.0,0.0,0.0);
    private long spasmCurrentTime = 0;
    private long pidTime = 0;
    public double difference,medie;
    public double medii[] = new double[10];
    public boolean rotating = false;
    public double realAngle, targetAngle;
    private double forward, right, clockwise;
    public void init() {
        pid.enable();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right
        slider = hardwareMap.get(DcMotor.class, "slider");
        slider2 = hardwareMap.get(DcMotor.class, "slider2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        claw = hardwareMap.servo.get("claw");
        touchy = hardwareMap.get(TouchSensor.class, "touchy");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
    }
    @Override
    public void start(){
        //imuT.start();
        Chassis.start();
        Systems.start();
    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop) {
                if(gamepad2.left_bumper) {
                    ok1 = false;
                    ok1 = true;
                }
                tru = true;

                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;
                /*
                pid.setPID(constants.pGyro,constants.iGyro,constants.dGyro);
                if(clockwise != 0.0){
                    correction = 0.0;
                    rotating = true;
                }
                else{
                    if((forward != 0.0 || right != 0.0) && Math.abs(medie) < 0.5) {
                        if (rotating) {
                            targetAngle = realAngle;
                            rotating = false;
                            pid.setSetpoint(targetAngle);
                        }
                        correction = pid.performPID(realAngle);
                    }
                    else{
                        correction = 0.0;
                    }
                }*/
                pmotorFL = -y - x - rx + correction;
                pmotorBL = -y + x - rx - correction;
                pmotorBR = -y - x + rx - correction;
                pmotorFR = -y + x + rx + correction;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }

                //SLOW-MOTION
                if (gamepad1.left_bumper) {
                    sm = 3;
                }
                else if (gamepad1.right_bumper) {
                    sm = 5;
                }
                else {
                    sm = 0.5;
                }
                if(sm==3){
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }
                else if(sm==5){
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }
                else{
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                }
            }
        }
    });
    private Thread imuT = new Thread(new Runnable() {
        double angle, lastAngle;
        int rotations;
        @Override
        public void run() {
            while(!stop){
                lastAngle = angle;
                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                difference = angle - lastAngle;
                if(Math.abs(difference) < 200){
                    for(i = 9; i > 0; i--){
                        medii[i] = medii[i-1];
                    }
                    medii[0] = difference;
                    for (i = 9; i > 0; i--) {
                        medie += medii[i];
                    }
                    medie /= 10.0;
                }
                if(lastAngle > 170 && angle < -170) rotations++;
                else if (lastAngle < -170 && angle > 170) rotations --;
                realAngle = angle + 360 * rotations;
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                pid.setPID(constants.kp,constants.ki,constants.kd);
                double armPower  = gamepad2.left_stick_y * 0.5;
                arm.setPower(armPower / ms);
                arm2.setPower(-armPower / ms );
                sliderPower  = gamepad2.right_stick_y;
                slider.setPower(-sliderPower / ms);
                slider2.setPower(sliderPower / ms);
                /*if(gamepad2.b) {
                    slider.setTargetPosition(-1680);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0.7);
                    while (slider.isBusy() && ok1 == true) ;
                    slider.setPower(0);
                    slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }*/
                /*if(gamepad2.x) {
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0.3);
                    while (slider.isBusy()&&ok1==true) ;
                    slider.setPower(0);
                    slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }*/

                if(gamepad2.right_bumper)
                    ms = 2;
                else if (gamepad2.left_bumper)
                    ms = 5;
                else ms = 0.5;

                /*if (gamepad2.a)
                {claw.setPosition(0.0);

                }
                if (gamepad2.y)
                    claw.setPosition(0.4);*/
                boolean a=gamepad2.a;
                if(a!=alast){
                    cn++;
                }
                alast = a;
                if(cn%4==2){
                    claw.setPosition(1);
                }
                else if(cn%4==0){
                    claw.setPosition(0);
                }
                if (touchy.isPressed()) {
                    touch=true;
                    slider.setTargetPosition(0);
                    slider2.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0.7);
                    slider2.setPower(0.7);
                    while (slider.isBusy() && ok1 == true) ;
                    slider.setPower(0);
                    slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    while (slider2.isBusy() && ok1 == true) ;
                    slider2.setPower(0);
                    slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                else { // Otherwise, run the motor
                    touch=false;
                }
                if(gamepad2.dpad_up){
                    pid.setSetpoint(-393);
                    pid.performPID(-393);
                }
            }
        }
    });
    public void stop(){stop = true;}
    public void loop(){
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Pozitie slider", slider.getCurrentPosition());
        telemetry.addData("Pozitie slider2", slider2.getCurrentPosition());
        telemetry.addData("Controller Values slider:", gamepad2.right_stick_y);
        telemetry.addData("Controller Values arm:", gamepad2.left_stick_y);
        telemetry.addData("Poz: ", poz);
        telemetry.addData("touch ", touch);
        telemetry.addData("inchis: ", cn);
        telemetry.addData("permisie: ", permisie);
        telemetry.addData("asdf: ", gamepad1.right_stick_y);
        telemetry.addData("thread: ", stop);
        telemetry.addData("ghera: ", claw.getPosition());
        telemetry.addData("last slider position:", lastsliderPower);
        telemetry.update();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

