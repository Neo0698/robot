package org.firstinspires.ftc.teamcode.auto;

import static java.nio.file.Files.move;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Cam;
import org.firstinspires.ftc.teamcode.robot.GamepadController;
//import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Position2;

import java.util.List;

// @Config
public class AutoOpMode extends LinearOpMode
{
    public static double speed = 0.6;
    public static double sideSpeed = 0.8;
    public static double turnSpeed = 0.5;
    public static int lineDetect = 16;
    public static int adjustAmount = 3;
    public static int TEST_VAR = 3;

    public Cam.Etage etage;
    public enum Team {
        BLUE,
        RED
    }

    public enum StartPosition {
        WAREHOUSE,
        CAROUSEL
    }


    public Team team;
    public StartPosition startPosition;

    public AutoOpMode (AutoOpMode.Team initTeam, AutoOpMode.StartPosition initStartPosition){
        team = initTeam;
        startPosition= initStartPosition;
    }

    //////////////////////////////////////////////////////// CLASS MEMBERS /////////////////////////////////////////////////////////

    // Elapsed game time tracking.
    // private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private Movement movement;
    private Arm arm;
    //private Intake intake;
    private Launcher launcher;
    private Position2 position;
    private GamepadController gamepad;
    private Cam object;

    // Variables storing last state of gamepad buttons
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private double lastXtime = 0.0;
    private double lastRuntime = 0.0;

    private boolean tempIntake = false;

    private double tempMotorSpeed = 0.0;
    private boolean lastLT = false;
    private boolean lastRT = false;
    private boolean lastRB = false;

    private double dpadTime;


    public enum Path {
        A,
        B,
        C,
    }

    public void move(int amount, double x, double y, double r) {
        int start = movement.getEncoder();
        movement.move(x,y,r);
        movement.apply();
        while (Math.abs(movement.getEncoder() - start) < amount) {
            if (!opModeIsActive()) {
                break;
            }
        }
        movement.reset();
        movement.apply();
    }

    public void sleepFor(double millis) {
        telemetry.addData("Sleeping for ", millis);
        telemetry.update();
        double startTime = runtime.time();
        while (runtime.time() - startTime < (millis / 1000)) {
            if (!opModeIsActive()) {
                break;
            }
        }
    }

    ///////////////////////////////////////////////////////// OPMODE METHODS /////////////////////////////////////////////////////////

    public void canard_getter(){

        try {
            float x = Cam.get_x();

            float y = Cam.get_y();

            //telemetry.addData("Position x", String.valueOf(x));
            //telemetry.addData("Position y", y);

            //telemetry.update();
            move(1200, 0, x, 0);
            while (y > 100) {
                move(1200, 0, 100, 0);
                y = Cam.get_y();
                if (y == 0) {
                    break;
                }

            }
        }catch(Exception e){
            while(true) {
                telemetry.addData("error", "error cannard getter");
                telemetry.update();
            }


        }


    }

    public void runOpMode() {
        //////////// INIT ///////////
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Initializing...", "");




        movement = new Movement(
                telemetry,
                runtime,
                hardwareMap.get(DcMotor.class, "front_left_drive"),
                hardwareMap.get(DcMotor.class, "front_right_drive"),
                hardwareMap.get(DcMotor.class, "back_left_drive"),
                hardwareMap.get(DcMotor.class, "back_right_drive"),
                false
        );

        object = new Cam(
                telemetry,
                runtime,
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );

        movement.changebrake(Movement.Brake.TRUE);
        telemetry.addData("Movement", "Initialized");


        position = new Position2(
                telemetry,
                null,
                null,
                null
        );

        telemetry.addData("Position", "Initialized");

        telemetry.update();

        waitForStart();
        //////////// RUN /////////////

        // Init
        telemetry.addData("Step", "starting");
        telemetry.update();
        object.activate();

        // Detect object
        Path path = null;
        int iter=0;
        int moove=0;
        Cam.CameraObject object_to_detect = Cam.CameraObject.UNSURE;

        double startCheckTime = runtime.time();
        //move(1200, -500, 0, 0);
        while ((object_to_detect == Cam.CameraObject.NONE || object_to_detect == Cam.CameraObject.UNSURE)) {

            iter+=1;
            object_to_detect = object.checkCamera();
            telemetry.addData("Seeing Object", object_to_detect);
            telemetry.update();
            moove=4;
            if(moove<3) {
                if(iter==10000){
                    int i=0;
                    while(i<10) {
                        if(object_to_detect == Cam.CameraObject.NONE || object_to_detect == Cam.CameraObject.UNSURE) {
                            move(1200, -100, 0, 0);
                            i += 1;
                        }else{
                            return;
                        }
                    }
                    moove+=1;
                }
                if (iter == 20000) {
                        iter=0;
                    move(1200, 0, 0, 35);
                    iter = 0;
                    moove+=1;
                }


            }
            if (!opModeIsActive()) {
                return;
            }
        }
        if(object_to_detect==Cam.CameraObject.DUCK) {
            telemetry.addData("start", "cannard getter");

            telemetry.update();
            // canard_getter();
        }else {


            Cam.Etage et = Cam.Etage.ETAGE_UNSURE;
            et = object.getPosition(startPosition);

            telemetry.addData("Etage: ", et);
            telemetry.update();


            switch (et) {
                case ETAGE_1:
                    //programme etage 1
                    break;
                case ETAGE_2:
                    //programme etage 2
                    break;
                case ETAGE_3:
                    //programme etage 3
                    break;
                case ETAGE_UNSURE:
                    //programme etage 1
                default:
                    //programme etage 1
                    break;
            }

            switch (startPosition) {
                case WAREHOUSE:
                    path = Path.B;
                    break;
                case CAROUSEL:
                    path = Path.A;
                    break;
                default:
                    path = Path.A;
                    break;
            }
        }

    }}