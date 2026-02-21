package org.firstinspires.ftc.teamcode.opModes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Assembly;

@Autonomous(name = "Auto Red (Far)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Red Far Start)")
@Configurable // Panels
public class autoRed12F extends autoBlue12F{
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_RED;
        ROT = 0;
    }
}
