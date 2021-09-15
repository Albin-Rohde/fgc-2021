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

@TeleOp(name="Main", group="Linear Opmode")
public class Main extends LinearOpMode {
  public boolean running = false;

  // utils
  private ElapsedTime runtime = new ElapsedTime();
  int gearLevel = 3;
  int posPerRound = 27;
  int posIncremt = posPerRound * gearLevel; // 81 //  27 * 3 = 81


  // Motors
  private DcMotor motor1 = null;
  private DcMotor motor2 = null;
  private DcMotor motor3 = null;
  private DcMotor motor4 = null;

  // Other devices
  private DistanceSensor rangeSensor = null;


  private void mapDevices() {
    motor1 = hardwareMap.get(DcMotor.class, "1");
    motor2 = hardwareMap.get(DcMotor.class, "2");
    motor3 = hardwareMap.get(DcMotor.class, "3");
    motor4 = hardwareMap.get(DcMotor.class, "4");
    rangeSensor = hardwareMap.get(DistanceSensor.class, "rs");
  }

  private void driveForward() {
    motor1.setPower(-1);
    motor2.setPower(1);
    motor3.setPower(-1);
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
    motor1.setPower(1);
    motor2.setPower(-1);
    motor3.setPower(1);
    motor4.setPower(-1);

    boolean keepLoop = true;
    boolean doneWithM1 = false;
    boolean doneWithM2 = false;

    int prevPosM1 = motor1.getCurrentPosition();
    int prevPosM2 = motor2.getCurrentPosition();
    while(keepLoop) {
      sleep(1);
      if (!doneWithM1 && (motor1.getCurrentPosition() > prevPosM1)) {
        motor1.setPower(0);
        doneWithM1 = true;
      }
      if (!doneWithM2 && (motor2.getCurrentPosition() < prevPosM2)) {
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
    waitForStart();
    running = true;
    int beginPos = motor2.getCurrentPosition();
    int laps = 0;

    while (opModeIsActive() && running) {
      driveForward();
      if (motor2.getCurrentPosition() >= beginPos + posIncremt) {
        beginPos += posIncremt;
        telemetry.addData("info", "time" + runtime);
        laps++;
        runtime.reset();
      }
      telemetry.addData("info", rangeSensor.getDistance(DistanceUnit.CM));
      telemetry.update();
      if (rangeSensor.getDistance(DistanceUnit.CM) < 190) {
        emergancyStop();
        sleep(100000000);
        running = false;
      }
    }
  }
}
