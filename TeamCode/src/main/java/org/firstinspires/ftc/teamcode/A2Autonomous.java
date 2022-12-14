package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous
public class A2Autonomous extends LinearOpMode {
    private OpenCvCamera webcam;
    private PachetelNouOpenCV pachetelNouOpenCV = new PachetelNouOpenCV();
    private double width, height;
    public DcMotorEx motorFR, motorFL, motorBR, motorBL;
    public DcMotorEx slider,slider2;
    int ok=0,pduck=0;
    public double lastTime;
    boolean a=true;

    private Servo claw;
    private static final String cub = "Cube";
    String varrez = "Dreapta";
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double Lpos = 0.7;
    //public Encoder aa;
    /*private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left*/

    private double lowerRuntime = 0;
    private double upperRuntime = 0;
    FunctiiDeAutonom f = new FunctiiDeAutonom(true);
    // Pink Range                                      Y      Cr     Cb
    /*public static Scalar scalarLowerYCrCb = new Scalar(0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(235.0, 235.0, 235.0);*/
    @Override
    public void runOpMode() throws InterruptedException {

        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        slider2 = hardwareMap.get(DcMotorEx.class, "slider2");

        claw = hardwareMap.servo.get("claw");

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pachetelNouOpenCV);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
            while (opModeIsActive()) {
                try{
                width = pachetelNouOpenCV.getRect().width;
                height = pachetelNouOpenCV.getRect().height;
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addData("Rectangle Width:", width);
                telemetry.addData("Rectangle Height:", height);
                telemetry.addData("Rectangle H/W:", height / width);
                if (height / width < 0.9) {
                    telemetry.addData("Rect", "3");
                } else if (height / width < 3) {
                    telemetry.addData("Rect", "1");
                } else {
                    telemetry.addData("Rect", "2");
                }
                telemetry.update();
            }
            catch(Exception E){
                height = 1;
                width = 1000;
            }
            }
            Autonom.start();
        while(!isStopRequested()){

        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            /*varrez = "Mijloc";
            if(varrez=="Dreapta"&&!isStopRequested()) {
                Translatare(130,0,0.5);
                kdf(200);
                Translatare(0,-245,0.5);
            }

            if(varrez == "Mijloc"&&!isStopRequested()) {
                Translatare(130,0,0.5);
                kdf(200);
                Translatare(0,-245,0.5);
                kdf(600);
                Translatare(-130,0,0.5);
            }

            if(varrez == "Stanga"&&!isStopRequested()) {
                Translatare(130,0,0.5);
                kdf(200);
                Translatare(0,-245,0.5);
                kdf(600);
                Translatare(-260,0,0.5);
            }*/
            f.gotoX(1000,1);
        }
    });
    public void testing(ContourPiepline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 235);
        crThreshHigh = inValues(crThreshHigh, 0, 235);
        cbThreshLow = inValues(cbThreshLow, 0, 235);
        cbThreshHigh = inValues(cbThreshHigh, 0, 235);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void RotireKindaSmooth(int poz, double power, int choice){
        if(choice%4==0) {
            motorFR.setTargetPosition(poz);

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFR.setPower(power);
        }
        else if(choice%4==1) {
            motorFL.setTargetPosition(poz);

            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFL.setPower(power);
        }
        else if(choice%4==2) {
            motorBR.setTargetPosition(poz);

            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBR.setPower(power);
        }
        else if(choice%4==3) {
            motorBL.setTargetPosition(poz);

            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBL.setPower(power);
        }
    }
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }
    void brat_to_the_top(){
        slider.setTargetPosition(-2400);//465
        slider2.setTargetPosition(-2520);//465
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
        slider2.setPower(1);
        while(slider.isBusy() && slider2.isBusy());
        slider.setPower(0);
        slider2.setPower(0);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void brat_retreat(){
        slider.setTargetPosition(0);//465
        slider2.setTargetPosition(0);//465
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
        slider2.setPower(1);
        while(slider.isBusy() && slider2.isBusy());
        slider.setPower(0);
        slider2.setPower(0);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void claw_pos(double claw_pos){
        claw.setPosition(claw_pos);
    }
    public void kdf(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime + t < System.currentTimeMillis()){

        }
    }
}
/*              |
                |
                |
                |
                |
________________|________________
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |

 */
