package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

@SuppressWarnings({"FieldCanBeLocal", "unused", "WeakerAccess"})
public class SkyBot extends TilerunnerGtoBot {
    private final String TAG = "SJH SkyRobot";

    public DcMotor _liftyBoi = null;
    //5202 series yellow jacket motor w/ 5.2:1 gear box has 145.6 cpr of output shaft
    //  1150 rpm (19.17 rps) no load speed
    //
    // Replaced with 13.7:1 gear box motor with 383.6 cpr
    // And Replaced with 3.7:1 gear box motor with 103.6 cpr
    //lift is made with 1:1 gear between motor output shaft and lead screw
    //
    //gobilda lead screw has 7.9in range, 2mm pitch, and 4 start
    //lead=2mm pitch * 4 start = 8 mm/lead screw rev
    //19.17rps * 8mm/rev = 153mm/s no load lift speed (6in/s)
    //201mm (7.9in) range in 201/153=1.31 s
    //201mm / 8mm/rev = 25.125rev
    //25.125rev * 145.6 cpr = 3658 counts

    private final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double LIFTER_INT_GEAR = 3.7; //13.7;  // 5.2;
    private final double LIFTER_CPOR = LIFTER_CPER * LIFTER_INT_GEAR; //383.6 //145.6 cnts/outShaftRev
    private final double LIFTER_EXT_GEAR = 1.0;
    private final double LIFTER_PITCH = 2.0; //2.0mm/pitch
    private final int LIFTER_STARTS = 4; //pitch/rev
    private final double LIFTER_LEAD = LIFTER_PITCH * LIFTER_STARTS; //8 mm/rev
    private final double LIFTER_CPMM = LIFTER_EXT_GEAR * LIFTER_CPOR / LIFTER_LEAD;
    private final double LIFTER_CPI = LIFTER_CPMM * 25.4;
    @SuppressWarnings("FieldCanBeLocal")
    private final int LIFTER_THRESH = (int) (0.3 * LIFTER_CPI);
    private final double LIFTER_STOW = 0.0;
    private final double LIFTER_GRAB = -5.5;
    private final double LIFTER_MOVE = -5.0;
    private final double LIFTER_REL1 = -2.0;
    private final double LIFTER_REL2 = 0.0;
    private final int LIFT_STOW_CNTS = (int) (LIFTER_CPI * LIFTER_STOW);
    private final int LIFT_GRAB_CNTS = (int) (LIFTER_CPI * LIFTER_GRAB);
    private final int LIFT_MOVE_CNTS = (int) (LIFTER_CPI * LIFTER_MOVE);
    private final int LIFT_REL1_CNTS = (int) (LIFTER_CPI * LIFTER_REL1);
    private final int LIFT_REL2_CNTS = (int) (LIFTER_CPI * LIFTER_REL2);

    public Servo lplatch = null;
    public Servo rplatch = null;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_STOW = 0.02;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_PRE = 0.32;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_GRAB = 0.78;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_STOW = 0.80;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_PRE = 0.50;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_GRAB = 0.04;

    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_STOW = 0.40;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_OPEN = 0.99;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_PRE = 0.6;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_GRAB = 0.32;


    private final int EXTND_COUNTS_PER_MOTOR_REV = 4;
    private final double EXTND_GEAR_ONE = 72;
    private final double EXTND_WHEEL_DIAM = 1.57;
    private final double EXTND_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE;
    private final double EXTND_SCALE = 1.0;
    public double EXTND_CPI = EXTND_CPR / (Math.PI * EXTND_WHEEL_DIAM) / EXTND_SCALE;
    private int MIN_EXTND_CNT = 0;
    private int ARM_EXT_STOW_POS = (int) (0.0 * EXTND_CPI);
    private int ARM_EXT_SNUG_POS = (int)  (7.0 * EXTND_CPI);
    private int ARM_EXT_STAGE_POS = (int) (9.0 * EXTND_CPI);
    private int ARM_EXT_GRAB_POS = (int) (14.0 * EXTND_CPI);
    private int ARM_EXT_DROP_POS = (int) (17.0 * EXTND_CPI);
    private int MAX_EXTND_CNT = (int) (18.0 * EXTND_CPI);

    int ARMROT_COUNTS_PER_MOTOR_REV = 4;
    double ARMROT_GEAR_ONE = 72.0;
    double ARMROT_CPR = ARMROT_COUNTS_PER_MOTOR_REV * ARMROT_GEAR_ONE;
    double ARMROT_CPD = ARMROT_CPR / 360.0;
//    private int ARM_ROT_FWD = (int) ( 90.0 * ARMROT_CPD);
//    private int ARM_ROT_RGT = (int) (  0.0 * ARMROT_CPD);
//    private int ARM_ROT_LFT = (int) (180.0 * ARMROT_CPD);
//    private int ARM_DRP_RGT = (int) ( 45.0 * ARMROT_CPD);
//    private int ARM_DRP_LFT = (int) (135.0 * ARMROT_CPD);
//    private int ARM_ROT_MIN = (int) (210.0 * ARMROT_CPD);
//    private int ARM_ROT_MAX = (int) (-30.0 * ARMROT_CPD);

    //following asumes 270 degree servo range
    public double ARM_ROT_RGT = 0.16;
    public double ARM_DRP_RGT = 0.32;
    public double ARM_ROT_FWD = 0.52;
    public double ARM_DRP_LFT = 0.72;
    public double ARM_ROT_LFT = 0.88;

    public static double ARM_GRB_RGT_L = 0.24;
    public static double ARM_GRB_RGT_R = 0.04;
    public static double ARM_GRB_LFT_L = 1.00;
    public static double ARM_GRB_LFT_R = 0.80;

    public double ARM_ROT_MIN = 0.0;
    public double ARM_ROT_MAX = 1.0;

    private DigitalChannel armIndexSensor = null;

    //    public  DcMotor  armRotate   = null;
    public Servo armRotate = null;
    public DcMotor armExtend = null;

    public SkyBot() {
        super();

        CAMERA_X_IN_BOT = 4.0f * (float) Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 3.0f * (float) Units.MM_PER_INCH;
    }

    public SkyBot(String name) {
        this();
        this.name = name;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor) {
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initSensors(initDirSensor);
        initArm();
        initHolder();
        initCapabilities();
    }

    @Override
    public void initCollectorLifter() //elev and stone gripper
    {
        initGripper();
        initLifter();
    }

    private double curRotSrvPos = 0.5;

    @Override
    public void initArm() //rotate, extend
    {
        RobotLog.dd(TAG, "GTO initArm");
        try {
            armExtend = hwMap.dcMotor.get("armExtend");
//            armRotate  = hwMap.dcMotor.get("armRotate");
            armRotate = hwMap.servo.get("armRotate");
            if (armRotate != null) curRotSrvPos = armRotate.getPosition();
//            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
            armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            capMap.put("arm", true);
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR in initArm\n" + e.toString());
        }
    }

    @Override
    public void initHolder() //Platform grabber
    {
        boolean success = true;
        try {
            rplatch = hwMap.servo.get("rplatch");
        } catch (Exception e) {
            return;
        }

        try {
            lplatch = hwMap.servo.get("lplatch");
        } catch (Exception e) {
            return;
        }

        capMap.put("holder", true);
    }

    @Override
    public void initSensors() {
        super.initSensors();
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initGripper() {
        try {
            gripper = hwMap.servo.get("gripper");

            //if extended range is wanted (and available) on servo,
            //keep block below
            //if standard range is wanted, comment block below
            if (gripper.getController() instanceof ServoControllerEx) {
                // Set the rotation servo for extended PWM range
                ServoControllerEx srvCntrlrEx =
                        (ServoControllerEx) gripper.getController();
                int gPort = gripper.getPortNumber();
                PwmControl.PwmRange range =
                        new PwmControl.PwmRange(500, 2500);
                srvCntrlrEx.setServoPwmRange(gPort, range);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR in initGripper\n" + e.toString());
            return false;
        }

        capMap.put("collector", true);
        return true;
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initLifter() {
        try {
            _liftyBoi = hwMap.dcMotor.get("liftyboi");

            _liftyBoi.setDirection(DcMotorSimple.Direction.FORWARD);
            //_liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _liftyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            _liftyBoi.setPower(0.0f);
            _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR in initLifter\n" + e.toString());
            return false;
        }

        capMap.put("lifter", true);
        return true;
    }

    public boolean isElevTouchPressed() {
        return false;
    }

    public boolean isArmTouchPressed() {
        return false;
    }

    private boolean lastExtndUseCnts = false;

    public void setExtend(double spd, boolean useCnts, boolean override) {
        double aspd = spd;

        if (armExtend == null) {
            RobotLog.ee(TAG, "ERROR in setExtend - armExtend null\n");
            return;
        }

        int curArmCounts = armExtend.getCurrentPosition();
        if (useCnts) {
            if (!lastExtndUseCnts) {
                RobotLog.dd(TAG, "Moving to arm pos %d", curArmCounts);
                armExtend.setTargetPosition(curArmCounts);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtend.setPower(aspd);
            }
        } else {
            armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (curArmCounts <= MIN_EXTND_CNT && !override) aspd =
                    Range.clip(aspd, 0, 1.0);
            if (curArmCounts >= MAX_EXTND_CNT && !override) aspd =
                    Range.clip(aspd, -1.0, 0.0);
            armExtend.setPower(aspd);
        }
        lastExtndUseCnts = useCnts;
    }

    public void setExtendPos(int targetPos) //targetPos in counts
    {
        final double EXT_THRESH = 40;

        if (armExtend == null) {
            RobotLog.ee(TAG, "armExtend is null - fix it");
            return;
        }

        RobotLog.dd(TAG, "armExtend to %d", targetPos);

        armExtend.setTargetPosition(targetPos);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double armExtSpd = 0.7;

        armExtend.setPower(armExtSpd);

        int curPos = armExtend.getCurrentPosition();
        while (op.opModeIsActive() && Math.abs(curPos - targetPos) > EXT_THRESH) {
            RobotLog.dd(TAG, "In armExtend.  targetpos=" + targetPos + " curPos=" +
                    curPos);
            curPos = armExtend.getCurrentPosition();
        }
        armExtend.setPower(0.0);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void putExtendAtStow() {
        setExtendPos(ARM_EXT_STOW_POS);
    }

    public void putExtendAtStage() {
        setExtendPos(ARM_EXT_STAGE_POS);
    }

    public void putExtendAtSnug() {
        setExtendPos(ARM_EXT_SNUG_POS);
    }

    public void putExtendAtGrab() {
        setExtendPos(ARM_EXT_GRAB_POS);
    }

    public void putExtendAtDrop()

    {
        setExtendPos(ARM_EXT_DROP_POS);
    }

    public void zeroArmExtend()
    {
        if(armExtend == null) return;
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean lastRotUseCnts = false;

    public void setRotate(double spd, boolean useCnts, boolean override)
    {
//        double aspd = spd;

        if (armRotate == null)
        {
            RobotLog.ee(TAG, "ERROR in setRotate - armRotate null\n");
            return;
        }

//        int curArmCounts = armRotate.getCurrentPosition();
//        if(useCnts)
//        {
//            if(!lastRotUseCnts)
//            {
//                RobotLog.dd(TAG, "Moving to arm pos %d", curArmCounts);
//                armRotate.setTargetPosition(curArmCounts);
//                armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armRotate.setPower(aspd);
//            }
//        }
//        else
//        {
//            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if(curArmCounts <= ARM_ROT_MIN && !override) aspd = 0.0;
//            if(curArmCounts >= ARM_ROT_MAX && !override) aspd = 0.0;
//            armRotate.setPower(aspd);
//        }
        lastRotUseCnts = useCnts;
    }

    public void setRotate (double scale)
    {
        double baseScale = 0.025;
        curRotSrvPos = armRotate.getPosition();
        curRotSrvPos = Range.clip(curRotSrvPos + baseScale * scale, -1.0, 1.0);
        RobotLog.dd(TAG,"Setting armRotate to %f", curRotSrvPos);
        armRotate.setPosition(curRotSrvPos);
    }

    //public void setRotatePos(int targetPos)
    public void setRotatePos(double targetPos)
    {
//        final double ROT_THRESH = 4;
//
        if(armRotate == null)
        {
            RobotLog.ee(TAG, "armRotate is null - fix it");
            return;
        }

        RobotLog.dd(TAG, "armRotate to %f", targetPos);
        armRotate.setPosition(targetPos);
//
//        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        double armRotSpd = 0.3;
//
//        armRotate.setTargetPosition(targetPos);
//        armRotate.setPower(armRotSpd);
//
//        while(op.opModeIsActive())
//        {
//            RobotLog.dd(TAG, "In armRotate.  targetpos=" + targetPos + " curPos=" +
//                    armExtend.getCurrentPosition());
//            if(Math.abs(armRotate.getCurrentPosition() - targetPos) < ROT_THRESH)
//            {
//                armRotate.setPower(0.0);
//                break;
//            }
//        }
//        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void putArmForward()   { setRotatePos(ARM_ROT_FWD); }
    public void putArmLeft()      { setRotatePos(ARM_ROT_LFT); }
    public void putArmRight()     { setRotatePos(ARM_ROT_RGT); }
    public void putArmHalfLeft()  { setRotatePos(ARM_DRP_LFT); }
    public void putArmHalfRight()
    {
        setRotatePos(ARM_DRP_RGT);
    }

    public void zeroArmRotate()
    {
//        if(armRotate == null) return;
//        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void openGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "openGripper %.2f", GRIPPER_OPEN);
        gripper.setPosition(GRIPPER_OPEN);
    }

    public void stowGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "stowGripper %.2f", GRIPPER_STOW);
        gripper.setPosition(GRIPPER_STOW);
    }

    public void stageGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "stageGripper %.2f", GRIPPER_PRE);
        gripper.setPosition(GRIPPER_PRE);
    }

    public void closeGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "closeGripper %.2f", GRIPPER_GRAB);
        gripper.setPosition(GRIPPER_GRAB);
    }

    public void setLiftPos(int targetPos)
    {
        if(_liftyBoi == null)
        {
            RobotLog.ee(TAG, "liftyboi is null - fix it");
            return;
        }

        RobotLog.dd(TAG, "liftyboi to %d", targetPos);

        _liftyBoi.setPower(0.0);
        _liftyBoi.setTargetPosition(targetPos);
        _liftyBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double liftSpd = 0.6;

        _liftyBoi.setTargetPosition(targetPos);
        _liftyBoi.setPower(liftSpd);

        ElapsedTime liftTimer = new ElapsedTime();
        double liftTimeLimit = 5.0;
        while(op.opModeIsActive() && liftTimer.seconds() < liftTimeLimit)
        {
            RobotLog.dd(TAG, "In lift.  targetpos=" + targetPos + " curPos=" +
                    _liftyBoi.getCurrentPosition());
            if(Math.abs(_liftyBoi.getCurrentPosition() - targetPos) < LIFTER_THRESH)
            {
                _liftyBoi.setPower(0.0);
                break;
            }
        }
        _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getLiftPos()
    {
        int pos = -9999;
        if(_liftyBoi != null) pos = _liftyBoi.getCurrentPosition();
        return pos;
    }

    public void threadedLiftStage()
    {
        class LiftRunnable implements Runnable
        {
            public void run()
            {
                putLiftAtGrab();
            }
        }

        LiftRunnable mr = new LiftRunnable();
        Thread liftThread = new Thread(mr, "liftThread");
        liftThread.start();
    }

    public void moveLift(double inches)
    {
        int curPos = _liftyBoi.getCurrentPosition();
        int cnts = (int) (inches*LIFTER_CPI);
        setLiftPos(curPos + cnts);
    }

    public void zeroLift()
    {
        if(_liftyBoi == null) return;
        _liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void putLiftAtGrab()
    {
        setLiftPos(LIFT_GRAB_CNTS);
    }

    public void putLiftAtStow()
    {
        setLiftPos(LIFT_STOW_CNTS);
    }

    public void putLiftAtMove()
    {
        setLiftPos(LIFT_MOVE_CNTS);
    }

    public void putLiftAtDrop()
    {
        setLiftPos(LIFT_REL1_CNTS);
    }

    public void putLiftAtLevel2()
    {
        setLiftPos(LIFT_REL2_CNTS);
    }

    public void putHolderAtStow()
    {
        if(lplatch != null) lplatch.setPosition(LPLATCH_STOW);
        if(rplatch != null) rplatch.setPosition(RPLATCH_STOW);
    }

    public void putHolderAtPre()
    {
        lplatch.setPosition(LPLATCH_PRE);
        rplatch.setPosition(RPLATCH_PRE);
    }

    public void putHolderAtGrab()
    {
        lplatch.setPosition(LPLATCH_GRAB);
        rplatch.setPosition(RPLATCH_GRAB);
    }
}
