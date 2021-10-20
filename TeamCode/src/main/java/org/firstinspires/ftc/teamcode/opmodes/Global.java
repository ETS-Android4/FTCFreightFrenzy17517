package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.ButtonActivatedModes.Duck;

public class Global {
    public LinearOpMode linearOpMode = null;
    public Global(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }
    private Duck duck = new Duck(linearOpMode);
    private Movement move = new Movement(linearOpMode);

    public void init(){
        duck.init();
        move.init();

    }
}
