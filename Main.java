package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Main", group="Linear Opmode")
public class Main extends LinearOpMode {
  private boolean running = false;
  private DcMotor Dclift = null;
  private Motor lift = null;
  private DcMotor Dcm1 = null;
  private DcMotor Dcm2 = null;
  private Motor m1 = null;
  private Motor m2 = null;
  // public DcMotor m2 = null;

  public void mapDevices() {
    // den här styr port 0
    this.Dclift = hardwareMap.get(DcMotor.class, "1");
    this.lift = new Motor(this.Dclift, false);
    
    this.Dcm1 = hardwareMap.get(DcMotor.class, "2");
    this.m1 = new Motor(this.Dcm1, false);
    
    this.Dcm2 = hardwareMap.get(DcMotor.class, "3");
    this.m2 = new Motor(this.Dcm2, false);
    // this.m2 = hardwareMap.get(DcMotor.class, "2");
  }

  @Override
  public void runOpMode() {
    waitForStart();
    mapDevices();
    this.running = true;
    while (opModeIsActive() && this.running) {
      
      
      
      if(gamepad1.dpad_up) {
        this.m1.setPower(1);
        this.m2.setPower(1);
      } else if(gamepad1.dpad_down) {
        this.m1.setPower(-1);
        this.m2.setPower(-1);
      } else {
        this.m1.setPower(0);
        this.m2.setPower(0);
      }
      
      
      
      
      
      if (gamepad1.a) {
        this.lift.setPower(1);
        // this.m2.setPower(-1);
      } else if (gamepad1.b) {
        this.lift.setPower(-1);
        // this.m2.setPower(1);
      } else {
        this.lift.setPower(0);
        // this.m2.setPower(0);
      }
    }
  }
}
