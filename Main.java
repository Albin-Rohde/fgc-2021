package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Motor;

@TeleOp(name="Main", group="Linear Opmode")
public class Main extends LinearOpMode {
  public boolean running = false;

  // utils
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime runtime2 = new ElapsedTime();
  int gearLevel = 1;
  int posPerRound = 27;
  int posIncremt = posPerRound * gearLevel; // 81 //  27 * 3 = 81


  // Motors
  private Motor motor1 = null;
  private Motor motor2 = null;
  private Motor motor3 = null;
  private Motor motor4 = null;
  private Servo servo1 = null;

  // Other devices
  private DistanceSensor rangeSensor = null;


  private Motor getMotor(String name, boolean reversed) {
    return new Motor(hardwareMap.get(DcMotor.class, name), reversed);
  }

  private void mapDevices() {
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    DcMotor m4 = null;
    m1 = hardwareMap.get(DcMotor.class, "1");
    m2 = hardwareMap.get(DcMotor.class, "2");
    m3 = hardwareMap.get(DcMotor.class, "3");
    m4 = hardwareMap.get(DcMotor.class, "4");

    this.motor1 = new Motor(m1, true);
    this.motor2 = new Motor(m2, false);
    this.motor3 = new Motor(m3, true);
    this.motor4 = new Motor(m4, false);

    rangeSensor = hardwareMap.get(DistanceSensor.class, "rs");
    servo1 = hardwareMap.get(Servo.class, "steer");
  }

  private void driveForward() {
    motor1.setPower(1);
    motor2.setPower(1);
    motor3.setPower(1);
    motor4.setPower(1);
  }

  private void stopMotor() {
    motor1.setPower(0);
    motor2.setPower(0);
  }
  /*
    The base motor supplies 27 positions per round
    with a level 3 gear that is 27 * 3 positions. 81 positions
   */

  private void emergancyStop() {
    motor1.setPower(-1);
    motor2.setPower(-1);
    motor3.setPower(-1);
    motor4.setPower(-1);

    boolean keepLoop = true;
    boolean doneWithM1 = false;
    boolean doneWithM2 = false;

    int prevPosM1 = motor1.getCurrentPosition();
    int prevPosM2 = motor2.getCurrentPosition();
    while(keepLoop) {
      sleep(1);
      if (!doneWithM1 && (motor1.getCurrentPosition() > prevPosM1 + 3)) {
        motor1.setPower(0);
        doneWithM1 = true;
      }
      if (!doneWithM2 && (motor2.getCurrentPosition() < prevPosM2 + 3)) {
        motor2.setPower(0);
        doneWithM2 = true;
      }
      if (doneWithM2 || doneWithM1) {
        motor3.setPower(0);
        motor4.setPower(0);
      }
      prevPosM1 = motor1.getCurrentPosition();
      prevPosM2 = motor2.getCurrentPosition();
    }
  }



  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    mapDevices();

    runtime.reset();
    runtime2.reset();
    waitForStart();

    int ii = 0;
    running = true;
    int beginPos = motor2.getCurrentPosition();
    int laps = 0;

    int nextTenTick = 0;
    int nextTenTickDiff = 0;

    int timeDelta[];
    int tickNumberArray[];
    // declaring array
    timeDelta = new int[60];
    tickNumberArray = new int[60];

    while (opModeIsActive() && running) {
      driveForward();
      /*
      runtime2.reset();
      int currentPosMotor2 = motor2.getCurrentPosition();
      nextTenTick = currentPosMotor2 + 10;

      if (currentPosMotor2 >= nextTenTick) {
        nextTenTickDiff = currentPosMotor2 - nextTenTick;
        nextTenTick = currentPosMotor2 + 10 - nextTenTickDiff;
        telemetry.addData("time", runtime2);
        telemetry.addData("tick", currentPosMotor2);
        ii++;
      }

      if (motor2.getCurrentPosition() >= beginPos + posIncremt) {
        beginPos += posIncremt;
        laps++;
        runtime.reset();
      }
       */
      if (rangeSensor.getDistance(DistanceUnit.CM) < 110) {
        emergancyStop();
        telemetry.update();
        sleep(100000000);
        running = false;
      }
    }
  }
}
