package org.firstinspires.ftc.teamcode.opModes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Assembly;

@Autonomous(name = "Auto Red (12 artifact no-overflow)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Red)")
@Configurable // Panels
public class autoRed12C extends autoBlue12C{
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_RED;
        ROT = 0;
    }
}
