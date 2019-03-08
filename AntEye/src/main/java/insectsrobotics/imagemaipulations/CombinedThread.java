package insectsrobotics.imagemaipulations;

import android.os.SystemClock;
import android.util.Log;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

import insectsrobotics.imagemaipulations.NavigationModules.WillshawModule;

import static java.lang.Thread.activeCount;
import static java.lang.Thread.sleep;

public class CombinedThread {
    private MainActivity app;
    private double distance = 0;
    private String tag = "CXEThread";

    public CombinedThread(MainActivity app){ this.app = app; }

    Runnable sequentialThread;
    Runnable CXEThread;
    {
        CXEThread = new Runnable(){
            @Override
            public void run(){
                try {
                    sleep(3000);
                } catch (Exception e){
                    e.printStackTrace();
                }

                CXE extendedcx = new CXE();

                int t0 = (int) SystemClock.elapsedRealtime();
                int t = (int) SystemClock.elapsedRealtime() - t0;
                int t_outbound = 40000;
                int t_inbound = 80000;

                //
                // Connectivity matrix declarations
                //
                SimpleMatrix tl2 = new SimpleMatrix(CX.n_tl2, 1);
                SimpleMatrix cl1 = new SimpleMatrix(CX.n_cl1, 1);
                SimpleMatrix tb1 = new SimpleMatrix(CX.n_tb1, 1);
                SimpleMatrix memory = new SimpleMatrix(CX.n_cpu4, 1);
                SimpleMatrix cpu4 = new SimpleMatrix(CX.n_cpu4, 1);
                SimpleMatrix cpu1 = new SimpleMatrix(CX.n_cpu1, 1);
                SimpleMatrix en = new SimpleMatrix(CXE.n_tb1, 1);
                SimpleMatrix ca = new SimpleMatrix(CXE.n_ca, 1);
                SimpleMatrix acc = new SimpleMatrix(2,1);

                //
                // Connectivity matrix instantiations
                //
                tl2.set(0);
                cl1.set(0);
                tb1.set(0);
                memory.set(.5);
                cpu4.set(0);
                cpu1.set(0);
                en.set(0);
                ca.set(0);
                acc.set(0);

                //
                // Motor settings - Different speeds account for motor differences
                //
                int leftSpeed = 15;
                int rightSpeed = 14;
                boolean motorReset = true; // Used to alter motor speeds

                int loopCounter = 0;

                while(t < t_outbound){
                    //
                    // Loop synchronisation delay
                    //
                    try {
                        sleep(600);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    //
                    // Update the directional neurons from the compass
                    //
                    tl2 = extendedcx.tl2Output(Math.toRadians(app.currentDegree));
                    cl1 = extendedcx.cl1Output(tl2);
                    tb1 = extendedcx.tb1Output(cl1, tb1);

                    //
                    // Displacement update
                    //
                    double speed = 4; // Arbitrary speed value for the CX
                    memory = extendedcx.cpu4Update(memory, tb1, speed);
                    cpu4 = extendedcx.cpu4Output(memory.copy());

                    //
                    // Collision avoidance update
                    //
                    acc = extendedcx.accUpdate(app.leftCAFlow, app.rightCAFlow);
                    ca = extendedcx.caOutput(tb1, acc);

                    //
                    // Start motors if signalled
                    //
                    if (motorReset){
                        motorReset = false;
                        Command.go(new double[]{leftSpeed, rightSpeed});
                        try{ sleep(1000); } catch (Exception e){ e.printStackTrace(); }
                    }

                    //
                    // Turning generation
                    //
                    cpu1 = extendedcx.cpu1Output(tb1, cpu4, en, ca, 1);

                    //
                    // Compute heading in regular intervals.
                    //
                    Log.i(tag, "CXE.collisionDetected: " + CXE.collisionDetected);
                    if (loopCounter >= 2){
                        Command.stop();
                        try{ sleep(1000); } catch (Exception e){e.printStackTrace();}

                        //
                        // Generate turn
                        //
                        app.CXmotor = extendedcx.motorOutput(cpu1);
                        app.CXnewHeading =
                                Math.toDegrees(
                                        Math.toRadians(app.currentDegree) +
                                                app.CXmotorChange * app.CXmotor
                                );
                        app.CXtheta = (app.CXnewHeading - app.currentDegree) % 360;

                        try{ Command.turnAround(app.CXtheta); } catch(Exception e){ e.printStackTrace(); }
                        loopCounter = 0;
                        motorReset = true;
                    }

                    loopCounter++;
                    t = (int) SystemClock.elapsedRealtime() - t0;
                }

                //
                // Between run stuff
                //
                try{
                    sleep(3000);
                } catch(Exception e) {
                    e.printStackTrace();
                }

                //
                // Turn around for ease.
                //
                try { Command.turnAround(170); } catch(Exception e) { e.printStackTrace(); }

                //
                // Update the directional neurons from the compass
                //
                tl2 = extendedcx.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = extendedcx.cl1Output(tl2);
                tb1 = extendedcx.tb1Output(cl1, tb1);

                //
                // Displacement update; really we just want to update the angle
                //
                memory = extendedcx.cpu4Update(memory, tb1, 0.0);
                cpu4 = extendedcx.cpu4Output(memory.copy());

                //
                // Inbound route with CXPI parameter reset
                //
                t0 = (int) SystemClock.elapsedRealtime();
                t = (int) SystemClock.elapsedRealtime() - t0;
                motorReset = false;
                loopCounter = 0;

                // Willshaw stuff isn't in yet
                // Prerequisite; initialise willshaw net, and turn to correct starting angle
                //
                /*boolean willshawInitialised = false;
                while (!willshawInitialised){
                    if (app.images_access) {
                        app.new_image = app.new SaveImages();
                        app.new_image.execute(app.processedDestImage, 7);
                        willshawInitialised = true;
                    }
                }*/

                //
                // Compute motor output, and turn
                //
                app.CXmotor = extendedcx.motorOutput(cpu1);
                app.CXnewHeading =
                        Math.toDegrees(
                                Math.toRadians(app.currentDegree) +
                                        app.CXmotorChange * app.CXmotor
                        );
                app.CXtheta = (app.CXnewHeading - app.currentDegree) % 360;

                //
                // Generate motor commands
                //
                try {

                    Command.turnAround(app.CXtheta);
                } catch (Exception e) {
                    e.printStackTrace();
                }


                while(t < t_inbound){
                    //
                    // Loop synchronisation delay
                    //
                    try {
                        sleep(600);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    //
                    // Update the directional neurons from the compass
                    //
                    tl2 = extendedcx.tl2Output(Math.toRadians(app.currentDegree));
                    cl1 = extendedcx.cl1Output(tl2);
                    tb1 = extendedcx.tb1Output(cl1, tb1);

                    //
                    // Update the flow
                    //
                    acc = extendedcx.accUpdate(app.leftCAFlow, app.rightCAFlow);
                    ca = extendedcx.caOutput(tb1, acc);

                    //
                    // Displacement update
                    //
                    double speed = 4; // Arbitrary speed value for the CX
                    memory = extendedcx.cpu4Update(memory, tb1, speed);
                    cpu4 = extendedcx.cpu4Output(memory.copy());

                    //
                    // Start motors if signalled
                    //
                    if (motorReset){
                        motorReset = false;
                        Command.go(new double[]{leftSpeed, rightSpeed});
                        try{ sleep(1000); } catch (Exception e){ e.printStackTrace(); }
                    }

                    //
                    // Turning generation
                    //
                    cpu1 = extendedcx.cpu1Output(tb1, cpu4, en, ca, 0);

                    //
                    // If a collision has been detected
                    //
                    Log.i(tag, "CXE.collisionDetected: " + CXE.collisionDetected);

                    if (loopCounter >= 2){
                        //
                        // Else if for inbound
                        //
                        loopCounter = 0;
                        Command.stop();
                        try { sleep(1000); } catch (Exception e) { e.printStackTrace(); }

                        //
                        // Generate target angle
                        //
                        app.CXmotor = extendedcx.motorOutput(cpu1);

                        app.CXnewHeading =
                                Math.toDegrees(
                                        Math.toRadians(app.currentDegree) +
                                                app.CXmotorChange * app.CXmotor
                                );

                        app.CXtheta = (app.CXnewHeading - app.currentDegree) % 360;

                        //
                        // Generate motor commands; if a turn should be generated
                        //
                        if (app.CXtheta > 1.5 || app.CXtheta < -1.5) {
                            try {
                                Command.turnAround(app.CXtheta);
                            } catch (Exception e) {
                                e.printStackTrace();
                            }
                        }

                        // Signal the motors to be restarted
                        motorReset = true;
                    }

                    //
                    // Print out the turning angle
                    //
                    try {
                        app.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                app.debugTextView.setText(String.format(
                                        "CXTheta: %f",
                                        app.CXtheta
                                ));
                            }
                        });
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    loopCounter++;
                    t = (int) SystemClock.elapsedRealtime() - t0;
                }

            }
        };
    }

    {
        sequentialThread = new Runnable() {
            @Override
            public void run() {
                try {
                    sleep(3000);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                //
                // Initiate CX and MB models
                //
                CX centralComplex = new CX();
                app.mushroom_body = new WillshawModule();

                int startTime = (int) SystemClock.elapsedRealtime();
                int currentTime = (int) SystemClock.elapsedRealtime() - startTime;
                int outboundTime = 25000;
                int inboundTime = 80000;

                //
                // Connectivity matrix declarations
                //
                SimpleMatrix tl2 = new SimpleMatrix(CX.n_tl2, 1);
                SimpleMatrix cl1 = new SimpleMatrix(CX.n_cl1, 1);
                SimpleMatrix tb1 = new SimpleMatrix(CX.n_tb1, 1);
                SimpleMatrix memory = new SimpleMatrix(CX.n_cpu4, 1);
                SimpleMatrix cpu4 = new SimpleMatrix(CX.n_cpu4, 1);
                SimpleMatrix cpu1 = new SimpleMatrix(CX.n_cpu1, 1);

                //
                // Connectivity matrix instantiations
                //
                tl2.set(0);
                cl1.set(0);
                tb1.set(0);
                memory.set(.5);
                cpu4.set(0);
                cpu1.set(0);

                //
                // Motor settings - Different speeds account for motor differences
                //
                int leftSpeed = 15;
                int rightSpeed = 14;
                boolean motorReset = true; // Used to alter motor speeds

                //
                // Flow collision avoidance settings
                //
                int leftAccumulator = 0; //Add only left values
                int rightAccumulator = 0; //Add only right values
                int accumulationThreshold = 5000; //Threshold for a value to be accumulated (ignore all others) (def = 5000)
                int reactionThreshold = 10000; //Value to be met for a reaction to be triggered. (need at most four readings) (def = 1000)
                int turnAngle; // Angle by which to turn
                int loopCounter = 0; // Loop counter to reset the accumulators

                //
                // Distance and interval measurements
                //
                int intervalStart = (int) SystemClock.elapsedRealtime();

                //
                // Outbound loop
                //
                while (currentTime < outboundTime) {
                    //
                    // Loop synchronisation delay
                    //
                    try {
                        sleep(600);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    // Compute flow difference between the sides
                    app.ca_flow_diff = app.leftCAFlow - app.rightCAFlow;

                    //
                    // Accumulate values if they are great enough
                    //
                    if (app.ca_flow_diff >= accumulationThreshold) {
                        leftAccumulator = leftAccumulator + (int) app.ca_flow_diff;
                    } else if (app.ca_flow_diff <= -(accumulationThreshold)) {
                        rightAccumulator = rightAccumulator + Math.abs((int) app.ca_flow_diff);
                    }

                    //
                    // Reset values if they are stale
                    //
                    if (loopCounter >= 2) {
                        leftAccumulator = 0;
                        rightAccumulator = 0;
                        loopCounter = 0;
                    }

                    //
                    // Update the directional neurons from the compass
                    //
                    tl2 = centralComplex.tl2Output(Math.toRadians(app.currentDegree));
                    cl1 = centralComplex.cl1Output(tl2);
                    tb1 = centralComplex.tb1Output(cl1, tb1);

                    //
                    // Displacement update
                    //
                    double speed = 4; // Arbitrary speed value for the CX
                    memory = centralComplex.cpu4Update(memory, tb1, speed);
                    cpu4 = centralComplex.cpu4Output(memory.copy());

                    try {
                        app.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                app.debugTextView.setText(String.format(
                                        "Speed: %f \n" +
                                                "Left CA flow: %f \n" +
                                                "Right CA flow: %f \n" +
                                                "Flow Difference: %f \n" +
                                                "Current Degree: %f"
                                        ,
                                        app.speed,
                                        app.leftCAFlow,
                                        app.rightCAFlow,
                                        app.ca_flow_diff,
                                        app.currentDegree
                                ));
                            }
                        });
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    //
                    // Reset motor speeds if required
                    //
                    if (motorReset) {
                        motorReset = false;
                        Command.go(new double[]{leftSpeed, rightSpeed});
                        try {
                            sleep(1000);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        intervalStart = (int) SystemClock.elapsedRealtime();
                    }

                    boolean left = false;
                    boolean right = false;

                    if (leftAccumulator >= reactionThreshold) {
                        right = true;
                    } else if (rightAccumulator >= reactionThreshold){
                        left = true;
                    }

                    //
                    // Turning generation
                    //
                    cpu1 = centralComplex.cpu1Output(tb1, cpu4);
                    app.CXmotor = centralComplex.motorOutput(cpu1);

                    if (left){
                        Command.go(new double[]{0, 0});
                        try { sleep(1000); } catch(Exception e){ e.printStackTrace(); }
                        try {
                            Command.turnAround(20);
                        } catch(Exception e) {
                            e.printStackTrace();
                        }

                        motorReset = true;
                        leftAccumulator = 0;
                        rightAccumulator = 0;
                    } else if (right){
                        Command.go(new double[]{0, 0});
                        try { sleep(1000); } catch(Exception e) { e.printStackTrace(); }
                        try {
                            Command.turnAround(-20);
                        } catch(Exception e) {
                            e.printStackTrace();
                        }

                        motorReset = true;
                        leftAccumulator = 0;
                        rightAccumulator = 0;
                    }

                    loopCounter++;
                    currentTime = (int) SystemClock.elapsedRealtime() - startTime;
                } // End of outbound route

                //
                // Make sure the robot is completely stopped
                //
                Command.go(new double[]{0, 0});
                try {
                    sleep(1000);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                //
                // UI information
                //
                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format(
                                    "Outbound run complete" +
                                            "\nDistance travelled: %f",
                                    distance
                            ));
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                try{
                    sleep(3000);
                } catch(Exception e) {
                    e.printStackTrace();
                }

                //
                // Turn around for ease.
                //
                try { Command.turnAround(170); } catch(Exception e) { e.printStackTrace(); }

                //
                // Update the directional neurons from the compass
                //
                tl2 = centralComplex.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = centralComplex.cl1Output(tl2);
                tb1 = centralComplex.tb1Output(cl1, tb1);

                //
                // Displacement update; really we just want to update the angle
                //
                memory = centralComplex.cpu4Update(memory, tb1, 0.0);
                cpu4 = centralComplex.cpu4Output(memory.copy());

                //
                // Inbound route with CXPI parameter reset
                //
                startTime = (int) SystemClock.elapsedRealtime();
                currentTime = (int) SystemClock.elapsedRealtime() - startTime;
                intervalStart = (int) SystemClock.elapsedRealtime();
                motorReset = false;
                distance = 0;
                loopCounter = 0;

                //
                // Prerequisite; initialise willshaw net, and turn to correct starting angle
                //
                boolean willshawInitialised = false;
                while (!willshawInitialised){
                    if (app.images_access) {
                        app.new_image = app.new SaveImages();
                        app.new_image.execute(app.processedDestImage, 7);
                        willshawInitialised = true;
                    }
                }

                //
                // Compute motor output, and turn
                //
                app.CXmotor = centralComplex.motorOutput(cpu1);
                app.CXnewHeading =
                        Math.toDegrees(
                                Math.toRadians(app.currentDegree) -
                                        app.CXmotorChange * app.CXmotor
                        );
                app.CXtheta = (app.CXnewHeading - app.currentDegree) % 360;

                //
                // Generate motor commands
                //
                try {

                    Command.turnAround(app.CXtheta);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                //
                // PI Inbound; Visual Learning of homeward and outward
                // route.
                //
                while ( !Util.isHome(memory) /*true /*currentTime < inboundTime*/) {
                    try {
                        sleep(600);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    //
                    // Reset motor speeds if required
                    //
                    if (motorReset) {
                        motorReset = false;
                        Command.go(new double[]{leftSpeed, rightSpeed});
                        try {
                            sleep(1000);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        intervalStart = (int) SystemClock.elapsedRealtime();
                    }

                    //
                    // Learn image if it is available
                    //
                    if (app.images_access) {
                        app.new_image = app.new SaveImages();
                        app.new_image.execute(app.processedDestImage, 5);
                    }

                    // Compute flow difference between the sides
                    app.ca_flow_diff = app.leftCAFlow - app.rightCAFlow;

                    //
                    // Compass update
                    //
                    tl2 = centralComplex.tl2Output(Math.toRadians(app.currentDegree));
                    cl1 = centralComplex.cl1Output(tl2);
                    tb1 = centralComplex.tb1Output(cl1, tb1);

                    //
                    // Displacement update
                    //
                    double speed = 4;
                    memory = centralComplex.cpu4Update(memory, tb1, speed);
                    cpu4 = centralComplex.cpu4Output(memory.copy());

                    //
                    // Time based navigational update; every two loops, turn if necessary.
                    //
                    if (loopCounter >= 2) {
                        Command.go(new double[] {0, 0});
                        try { sleep(1000); } catch (Exception e) { e.printStackTrace(); }

                        //
                        // Generate target angle
                        //
                        cpu1 = centralComplex.cpu1Output(tb1, cpu4);
                        app.CXmotor = centralComplex.motorOutput(cpu1);

                       /* app.CXnewHeading = Math.toDegrees(
                                (Math.toRadians(app.currentDegree) + app.CXmotor + Math.PI)
                                        %(2.0 * Math.PI) - Math.PI
                        );*/


                        app.CXnewHeading =
                                Math.toDegrees(
                                        Math.toRadians(app.currentDegree) +
                                                app.CXmotorChange * app.CXmotor
                                );

                        app.CXtheta = (app.CXnewHeading - app.currentDegree) % 360;

                        //
                        // Print out the turning angle
                        //
                        try {
                            app.runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    app.debugTextView.setText(String.format(
                                            "CXTheta: %f",
                                            app.CXtheta
                                    ));
                                }
                            });
                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                        //
                        // Generate motor commands; if a turn should be generated
                        //
                        if (app.CXtheta > 1.5 || app.CXtheta < -1.5) {
                            try {
                                Command.turnAround(app.CXtheta);
                            } catch (Exception e) {
                                e.printStackTrace();
                            }
                        }

                        // Restart motors; will reset time interval too.
                        motorReset = true;
                    }

                    // Update current time
                    currentTime = (int) SystemClock.elapsedRealtime() - startTime;
                    loopCounter++;
                    if (Util.isHome(memory)){
                        try {
                            app.runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    app.debugTextView.setText(String.format(
                                            "I'm home!"
                                    ));
                                }
                            });
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    }
                }
            } // Run
        };
    }
}
