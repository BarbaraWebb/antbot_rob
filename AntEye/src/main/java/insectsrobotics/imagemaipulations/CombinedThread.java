package insectsrobotics.imagemaipulations;

import android.os.SystemClock;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

import insectsrobotics.imagemaipulations.NavigationModules.WillshawModule;

import static java.lang.Thread.sleep;

public class CombinedThread {
    private MainActivity app;
    private double distance = 0;

    public CombinedThread(MainActivity app){ this.app = app; }

    Runnable sequentialThread;

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
                int outboundTime = 30000;
                int inboundTime = outboundTime;

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
                        leftAccumulator += (int) app.ca_flow_diff;
                    } else if (app.ca_flow_diff <= -(accumulationThreshold)) {
                        rightAccumulator += Math.abs((int) app.ca_flow_diff);
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

                    // Check to see if one of the accumulators exceeds the threshold
                    boolean turn =
                            (leftAccumulator >= reactionThreshold) ||
                                    (rightAccumulator >= reactionThreshold);
                    

                    if (turn) {
                        // If leftAcc > rightAcc set the angle to 20deg, else, -20deg
                        turnAngle = (leftAccumulator > rightAccumulator) ? 20 : -20;

                        // Compute the distance. We assume the robot travels one "unit" per second
                        // that it is mobile.
                        distance = ((int) SystemClock.elapsedRealtime() - intervalStart) / 1000;

                        // Halt the robot
                        Command.go(new double[]{0, 0});

                        //
                        // Command delay
                        //
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
                                            "Distance travelled: %f",
                                            distance
                                    ));
                                }
                            });
                        } catch (Exception e) {
                            e.printStackTrace();
                        }


                        // Set the flag to restart the motors on the next iteration
                        motorReset = true;

                        //
                        // Update the directional neurons from the compass
                        //
                        tl2 = centralComplex.tl2Output(Math.toRadians(app.currentDegree));
                        cl1 = centralComplex.cl1Output(tl2);
                        tb1 = centralComplex.tb1Output(cl1, tb1);

                        //
                        // Displacement update
                        //
                        memory = centralComplex.cpu4Update(memory, tb1, distance);
                        cpu4 = centralComplex.cpu4Output(memory.copy());

                        //
                        // Turning generation
                        //
                        cpu1 = centralComplex.cpu1Output(tb1, cpu4);
                        app.CXmotor = centralComplex.motorOutput(cpu1);

                        //
                        // Perform the avoidance turn
                        //
                        try {
                            Command.turnAround(turnAngle);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                        // Reset the accumulators and interval distance
                        leftAccumulator = 0;
                        rightAccumulator = 0;
                        distance = 0;
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
                while (currentTime < inboundTime) {
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
                    // Time based navigational update
                    //
                    if (loopCounter >= 2) {
                        //
                        // Update distance information; again, one unit per second travelled
                        //
                        int interval = (int) SystemClock.elapsedRealtime() - intervalStart;
                        distance = (interval / 1000);

                        Command.go(new double[] {0, 0});
                        try { sleep(1000); } catch (Exception e) { e.printStackTrace(); }

                        //
                        // Compass update
                        //
                        tl2 = centralComplex.tl2Output(Math.toRadians(app.currentDegree));
                        cl1 = centralComplex.cl1Output(tl2);
                        tb1 = centralComplex.tb1Output(cl1, tb1);

                        //
                        // Displacement update
                        //
                        memory = centralComplex.cpu4Update(memory, tb1, distance);
                        cpu4 = centralComplex.cpu4Output(memory.copy());

                        //
                        // Generate target angle
                        //
                        cpu1 = centralComplex.cpu1Output(tb1, cpu4);
                        app.CXmotor = centralComplex.motorOutput(cpu1);
                        app.CXnewHeading =
                                Math.toDegrees(
                                        Math.toRadians(app.currentDegree) -
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
                                            distance
                                    ));
                                }
                            });
                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                        //
                        // Generate motor commands
                        //
                        try {
                            Command.turnAround(app.CXtheta);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                        // Restart motors; will reset time interval too.
                        motorReset = true;
                    }

                    // Update current time
                    currentTime = (int) SystemClock.elapsedRealtime() - startTime;
                    loopCounter++;
                }
            } // Run
        };
    }
}
