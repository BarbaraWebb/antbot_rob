package insectsrobotics.imagemaipulations;


import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.hardware.SensorEventListener;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.os.SystemClock;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.app.AlertDialog;
import android.content.DialogInterface;

import org.ejml.simple.SimpleMatrix;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import insectrobotics.broadcastlibrary.BroadcastValues;
import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Broadcast;
import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Receive;
import insectsrobotics.imagemaipulations.NavigationModules.WillshawModule;
import insectsrobotics.imagemaipulations.NavigationModules.RealWillshawModule;

import static java.lang.Thread.sleep;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.INTER_LANCZOS4;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.arrowedLine;
import static org.opencv.imgproc.Imgproc.goodFeaturesToTrack;
import static org.opencv.imgproc.Imgproc.remap;
import static org.opencv.imgproc.Imgproc.resize;
import static org.opencv.video.Video.calcOpticalFlowFarneback;
import static org.opencv.video.Video.calcOpticalFlowPyrLK;


//
// The old thread code must be kept in its original form, however, in MainActivity it is taking
// up too much space, so it is moved here. This class will extend MainActivity with the old
// runnables from previous years of this project. The class will be instantiated as a member in
// MainActivity so these threads can be accessed.
//
public class OldThreadCode{
    MainActivity app;
    public OldThreadCode(insectsrobotics.imagemaipulations.MainActivity app){
        this.app = app;
    }


     Runnable CXthread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Log.i(flowTag, "" + xPosOF + "," + yPosOF+","+orientation)
            CX c = new CX();
            Integer T_outbound = 40;
            Integer T_inbound = 30;//55;
            Integer T = T_inbound + T_outbound;

            SimpleMatrix tl2 = new SimpleMatrix(CX.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX.n_cpu4, 1);
            memory.set(.5);
            SimpleMatrix cpu4 = new SimpleMatrix(CX.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX.n_cpu1, 1);
            cpu1.set(0);

            app.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            app.CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-app.startTime;
            while (currentTime<T_outbound*1000) {
                app.CURRiteration++;
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format("outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\ncurrent iteration: %d"+
                                            "\ncurrent direction: %f",
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    app.CURRiteration,
                                    app.currentDegree));
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                // OUTBOUND ROUTE
                t = (int) SystemClock.elapsedRealtime() - t0;

                String direction = "not sure";
                if(t>300) {
                    if (currentTime < 3500) {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    // based on odometry
                    //app.ANT_SPEED = ()

                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction);
                    Log.d(app.TAG, Util.printMemory(memory));
                }
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, app.ANT_SPEED);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                app.CXmotor = c.motorOutput(cpu1);
                Util.writeToFile(
                        memory,
                        app.leftCXFlow,
                        app.rightCXFlow,
                        "run_1",
                        app.frame_rate_cx,
                        app.CURRiteration,
                        app.currentDegree
                );
                currentTime = (int) SystemClock.elapsedRealtime()-app.startTime;
            }

            // turn to offset pull, and allow network to redirect antbot
            Command.go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            try{
                Command.turnAround(45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currentTime = (int)SystemClock.elapsedRealtime()-t0;

            app.startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            Boolean algorithm_not_setted = true;

            while (algorithm_not_setted){
                if (app.images_access){
                    app.new_image = app.new SaveImages();
                    app.new_image.execute(app.processedDestImage, 7);
                    algorithm_not_setted = false;
                }
            }

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){
                app.CURRiteration++;
                try {
                    sleep(900);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if (app.images_access) {
                    app.new_image = app.new SaveImages();
                    app.new_image.execute(app.processedDestImage, 5);
                }

                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, app.ANT_SPEED);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                app.CXmotor = c.motorOutput(cpu1);
                Util.writeToFile(
                        memory,
                        app.leftCXFlow,
                        app.rightCXFlow,
                        "run_1",
                        app.frame_rate_cx,
                        app.CURRiteration,
                        app.currentDegree
                );

                // ----- ANTBOT DRIVING--------
                app.CXnewHeading = Math.toDegrees(Math.toRadians(app.currentDegree) - app.CXmotorChange * app.CXmotor);
                app.CXtheta = (app.CXnewHeading - app.currentDegree)%360;

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if(t > 300){
                    // speed in dm/sec
                    if (app.CXtheta<-1.5){
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (app.CXtheta>1.5){
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction + "\nwant to go to " + app.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(app.TAG, Util.printMemory(memory));
                }

                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d"+
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\nWant to go to: %f"+
                                            "\nSo turning of: %f",
                                    app.CURRiteration,
//                                    app.currentDegree,
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    app.CXnewHeading,
                                    app.CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - app.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                app.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        app.debugTextView.setText(String.format("The PI Run is over! "+
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));

                        // Show the "Put Robot Back to Food" Alert Dialog!
                        app.showDialog();
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    Runnable CXMBthread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Log.i(flowTag, "" + xPosOF + "," + yPosOF+","+orientation)

            Integer T_outbound = 40;
            Integer T_inbound = 30;//55;
            Integer T = T_inbound + T_outbound;

            SimpleMatrix tl2 = new SimpleMatrix(CX_MB.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX_MB.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX_MB.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX_MB.n_cpu4, 1);
            memory.set(.5);
            SimpleMatrix cpu4 = new SimpleMatrix(CX_MB.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX_MB.n_cpu1, 1);
            cpu1.set(0);

            SimpleMatrix en = new SimpleMatrix(CX_MB.n_tb1, 1);
            cpu1.set(0);

            app.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            app.CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-app.startTime;
            while (currentTime<T_outbound*1000) {
                app.CURRiteration++;
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format("outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\ncurrent iteration: %d"+
                                            "\ncurrent direction: %f",
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    app.CURRiteration,
                                    app.currentDegree));
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                // OUTBOUND ROUTE
                t = (int) SystemClock.elapsedRealtime() - t0;

                String direction = "not sure";
                if(t>300) {
                    if (currentTime < 3500) {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    // based on odometry
                    //app.ANT_SPEED = ()

                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction);
                    Log.d(app.TAG, Util.printMemory(memory));
                }
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = app.cxmb.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = app.cxmb.cl1Output(tl2);
                tb1 = app.cxmb.tb1Output(cl1, tb1);

//                String tb1_str = "";
//                for(int i = 0; i < 8; i++){
//                    tb1_str += tb1.get(i, 0) + " ";
//                }
//                LogToFileUtils.write(tb1_str);

                // ------ DISPLACEMENT UPDATE -----
                memory = app.cxmb.cpu4Update(memory, tb1, app.ANT_SPEED);
                cpu4 = app.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = app.cxmb.cpu1Output(tb1, cpu4, en, true);
                app.CXmotor = app.cxmb.motorOutput(cpu1);
                Util.writeToFile(
                        memory,
                        app.leftCXFlow,
                        app.rightCXFlow,
                        "run_1",
                        app.frame_rate_cx,
                        app.CURRiteration,
                        app.currentDegree
                );

                currentTime = (int) SystemClock.elapsedRealtime()-app.startTime;
            }

            // turn to offset pull, and allow network to redirect antbot
            Command.go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            try{
                Command.turnAround(45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currentTime = (int)SystemClock.elapsedRealtime()-t0;

            app.startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            app.stored_memory = memory.copy();

            Boolean algorithm_not_setted = true;

            while (algorithm_not_setted){
                if (app.images_access){
                    app.new_image = app.new LearnDirectionImages();
                    app.new_image.execute(app.processedDestImage, 7);
                    algorithm_not_setted = false;
                }
            }

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){
                app.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //------   COMPASS UPDATE  -----
                tl2 = app.cxmb.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = app.cxmb.cl1Output(tl2);
                tb1 = app.cxmb.tb1Output(cl1, tb1);

                if (app.images_access) {
                    app.new_image = app.new LearnDirectionImages();
                    app.new_image.execute(app.processedDestImage, 5, tb1);
                }

                // ------ DISPLACEMENT UPDATE -----
                memory = app.cxmb.cpu4Update(memory, tb1, app.ANT_SPEED);
                cpu4 = app.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = app.cxmb.cpu1Output(tb1, cpu4, en, true);
                app.CXmotor = app.cxmb.motorOutput(cpu1);
                Util.writeToFile(
                        memory,
                        app.leftCXFlow,
                        app.rightCXFlow,
                        "run_1",
                        app.frame_rate_cx,
                        app.CURRiteration,
                        app.currentDegree
                );

                // ----- ANTBOT DRIVING--------
                app.CXnewHeading = Math.toDegrees(Math.toRadians(app.currentDegree) - app.CXmotorChange * app.CXmotor);
                app.CXtheta = (app.CXnewHeading - app.currentDegree)%360;

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if(t > 300){
                    // speed in dm/sec
                    if (app.CXtheta<-1.5){
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (app.CXtheta>1.5){
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction + "\nwant to go to " + app.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(app.TAG, Util.printMemory(memory));
                }

                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d"+
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\nWant to go to: %f"+
                                            "\nSo turning of: %f",
                                    app.CURRiteration,
//                                    app.currentDegree,
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    app.CXnewHeading,
                                    app.CXtheta));
                            //Toast.makeText(getApplication(), "famialarity:" + familarity, Toast.LENGTH_SHORT);
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - app.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                app.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        app.debugTextView.setText(String.format("The PI Run is over! "+
                                "\nPlease Push the \"yes\" button when you are ready."));
                        // Show the "Put Robot Back to Food" Alert Dialog!
                        app.showDialog();
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };


    Runnable CXHolonomicThread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            CX_H_Pontin c = new CX_H_Pontin();
            Integer T_outbound = 40;            // outbound for 40 sec
            Integer T_inbound = 55;             // inbound for 55 sec
            Integer T = T_inbound + T_outbound; // total run

            SimpleMatrix tl2 = new SimpleMatrix(CX.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX.n_cpu4, 1);
            memory.set(.5);
            SimpleMatrix cpu4 = new SimpleMatrix(CX.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX.n_cpu1, 1);
            cpu1.set(0);

            int startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            app.CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-startTime;

            while (currentTime<T_outbound*1000) {
                app.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // print output to screen - useful for DEBUG
                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format(
                                    "outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f" +
                                            "\nFrameRate: %f",
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    app.frame_rate_cx
                                    )
                            );
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }

                // OUTBOUND ROUTE
                t = (int) SystemClock.elapsedRealtime() - t0;

                // hard coded outbound route
                String direction = "";
                if(t>300) {
                    if (currentTime < 3500) {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                }
                t0 = (int) SystemClock.elapsedRealtime();
                Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction);
                Log.d(app.TAG, Util.printMemory(memory));
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                //---- Speed retrieval ------------
                SimpleMatrix flow = new SimpleMatrix(new double[][]{{app.leftCXFlow},{app.rightCXFlow}});
                SimpleMatrix tn1 = c.tn1Output(flow);
                SimpleMatrix tn2 = c.tn2Output(flow);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, tn1, tn2);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                app.CXmotor = c.motorOutput(cpu1);
                currentTime = (int) SystemClock.elapsedRealtime()-app.startTime;
                Util.writeToFile(
                        memory,
                        app.leftCXFlow,
                        app.rightCXFlow,
                        "run",
                        app.frame_rate_cx,
                        app.CURRiteration,
                        app.currentDegree
                );
            }

            // offset turn, to allow network to redirect antbot
            Command.go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            try{
                Command.turnAround(45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currentTime = (int)SystemClock.elapsedRealtime()-t0;

            app.startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){

                app.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                //---- Speed retrieval ------------
                SimpleMatrix flow = new SimpleMatrix(new double[][]{{1.},{1.}});
                SimpleMatrix tn1 = c.tn1Output(flow);
                SimpleMatrix tn2 = c.tn2Output(flow);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, tn1, tn2);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                app.CXmotor = c.motorOutput(cpu1);
                Util.writeToFile(
                        memory,
                        app.leftCXFlow,
                        app.rightCXFlow,
                        "run",
                        app.frame_rate_cx,
                        app.CURRiteration,
                        app.currentDegree
                );

                // ----- ANTBOT DRIVING--------

                // new direction based on memory
                app.CXnewHeading = Math.toDegrees(
                        (Math.toRadians(app.currentDegree) + app.CXmotor + Math.PI)
                                %(2.0 * Math.PI) - Math.PI
                );

                // given new direction, turn a litte right/left to approach it
                app.CXtheta = ((app.CXnewHeading - app.currentDegree) % 360);

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "";
                if(t > 300){
                    // speed in dm/sec
                    if (app.CXtheta < -1.5){
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (app.CXtheta>1.5){
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction + "\nwant to go to " + app.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(app.TAG, Util.printMemory(memory));
                }

                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            float frame_rate;
                            if (app.currentFrameTime-app.prevFrameTime != 0){
                                frame_rate = 1/((app.currentFrameTime-app.prevFrameTime)*1000);
                            } else {
                                frame_rate = 0;
                            }
                            app.debugTextView.setText(String.format(
                                    "iutbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f" +
                                            "\nFrameRate: %f",
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    frame_rate
                                    )
                            );
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                currentTime = (int) SystemClock.elapsedRealtime() - app.startTime;

            }

            Command.go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                app.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        app.debugTextView.setText(String.format("The Run is over! "+
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };


    // Monitoring time to stop outbound PI and start Inbound
    Runnable startInbound = new Runnable() {
        @Override
        public void run() {
            try {
                sleep(app.integratorRunTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    };

    //
    // Simple optical flow avoidance routine
    //
    Runnable opticalFlowAvoidance = new Runnable() {
        @Override
        public void run() {
            //Should really be worked in with existing PI stuff
            //This version of the thread is for testing the flow filtering collision avoidance
            try {
                sleep(3000);
            } catch (Exception e){
                e.printStackTrace();
            }

            Util.log(app.log_file, "CA run started");
            boolean avoid = false; //Is the robot currently avoiding an obstacle?
            boolean stop = false; //Halt flag
            boolean initialise = true; //Loop initialisation flag
            int start_time = (int) SystemClock.elapsedRealtime(); //Start time for the thread
            int current_time = (int) SystemClock.elapsedRealtime() - start_time; //Current thread time
            int t_move_start = 0; //Time that a saccade started

            //Accumulation time intervals
            int t_interval_start = current_time; //Start of the time interval
            int t_delta = (int) SystemClock.elapsedRealtime() - current_time; //Time interval

            //Left and right motor speeds
            int lft_speed = 15;
            int rgt_speed = 14;

            //Accumulators for left and right flow
            int dual_accumulator = 0; //Add both values and see if there's significant bias
            int left_accumulator = 0; //Add only left values
            int right_accumulator = 0; //Add only right values
            int accumulation_threshold = 5000; //Threshold for a value to be accumulated (ignore all others) (def = 5000)
            int reaction_threshold = 10000; //Value to be met for a reaction to be triggered. (need at most four readings) (def = 1000)
            int turn = 20;

            boolean reaction_taken = false;
            int reaction_time;

            int loop_count = 0;


            //While
            while ( !stop && (current_time <= 30000) ){
                try { sleep(600); } catch ( Exception e ){ e.printStackTrace(); }

                //Print out debug information, catch exceptions
                app.ca_flow_diff = app.leftCAFlow - app.rightCAFlow; //If positive, left detection, else right detection

                //Flow accumulators
                if ( app.ca_flow_diff >= accumulation_threshold ){ left_accumulator = left_accumulator + (int) app.ca_flow_diff; }
                else if ( app.ca_flow_diff <= -accumulation_threshold ){ right_accumulator = right_accumulator + Math.abs((int) app.ca_flow_diff); }

                if ( Math.abs(app.ca_flow_diff) >= accumulation_threshold ){ dual_accumulator = dual_accumulator + (int) app.ca_flow_diff; }

                //If accumulation time limit reached, reset timer and accumulators
                if ( loop_count >= 2/*t_delta >= 2000 */){
                    left_accumulator = 0;
                    right_accumulator = 0;
                    dual_accumulator = 0;
                    loop_count = 0; //Reset loop counter
                    t_interval_start = (int) SystemClock.elapsedRealtime();
                }

                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run(){
                            app.debugTextView.setText(String.format(Locale.ENGLISH,
                                    "Speed: %f \n" +
                                            "Left CA flow: %f \n" +
                                            "Right CA flow: %f \n" +
                                            "Flow Difference: %f \n"
                                    ,
                                    app.speed,
                                    app.leftCAFlow,
                                    app.rightCAFlow,
                                    app.ca_flow_diff
                            ));
                        }
                    });
                } catch ( Exception e ){
                    e.printStackTrace();
                }
                //End printing

                //Initialise if 1st iteration (re-used to reset speeds mid-run)
                if ( initialise ){
                    initialise = false;
                    Command.go( new double[] { lft_speed, rgt_speed } );
                    try{ sleep(1000); } catch ( Exception e ){ e.printStackTrace(); } //Command delay
                    t_interval_start = (int) SystemClock.elapsedRealtime(); //Start the time interval for accumulation
                }

                //Figure out which way to turn
                boolean left = false;
                boolean right = false;

                //Dual decision (contiguous results)
                //if ( dual_accumulator >= reaction_threshold ){ right = true; }
                //else if ( dual_accumulator <= -reaction_threshold ){ left = true; }



                //Uncomment to use the individual method for accumulation.
                //Individual decision
                if ( left_accumulator >= reaction_threshold ){ right = true; }
                else if ( right_accumulator >= reaction_threshold ){ left = true; }
                //Left accumulator keeps track of left flow and so should trigger a right turn
                //Right accumulator keeps track of right flow and so should trigger a left turn


                if ( left && (!avoid) ){ //Left turn required
                    Log.i("CA:", "Left turn triggered");
                    if ( !reaction_taken ){
                        reaction_taken = true;
                        reaction_time = (int) SystemClock.elapsedRealtime() - t_interval_start;
                        //Reaction time in milliseconds. Print to stats file
                        StatFileUtils.write("OF", "RCTIME", "Reaction time: " + reaction_time + "; for speeds {" + lft_speed + ", " + rgt_speed + "}");
                    }

                    Command.go( new double[]{0, 0});
                    try { sleep(1000); } catch( Exception e ){ e.printStackTrace(); }
                    try {
                        Command.turnAround(turn);
                    }catch(Exception e){ e.printStackTrace(); }
                    /*
                    lft_speed = 10; //Left turn
                    rgt_speed = 100; //Left turn*/
                    initialise = true; //Re-call the go command to trigger the speed change
                    avoid = true; //Flag so we know to ignore this code snippet when avoiding
                    dual_accumulator = 0;
                    left_accumulator = 0;
                    right_accumulator = 0;
                    t_move_start = current_time; //Time the saccade started

                } else if ( right && (!avoid) ){ //Right turn required
                    Log.i("CA:", "Right turn triggered");
                    Command.go( new double[]{0, 0});
                    try { sleep(1000); } catch( Exception e ){ e.printStackTrace(); }
                    try {
                        Command.turnAround(-turn);
                    }catch(Exception e){ e.printStackTrace(); }
                    /*lft_speed = 100; //Right turn
                    rgt_speed = 10; //Right turn*/
                    initialise = true; //Re-call the go command to trigger the speed change
                    avoid = true; //Flag so we know to ignore this code snippet when avoiding
                    dual_accumulator = 0;
                    left_accumulator = 0;
                    right_accumulator = 0;
                    t_move_start = current_time; //Time the saccade started

                } else if ( avoid && ((current_time - t_move_start ) >= 500)){
                    //Make adjustments in half second intervals
                    lft_speed = 15; //Back to default
                    rgt_speed = 14; //Back to default
                    avoid = false; //No longer avoiding an obstacle
                    initialise = true; //Need to reset robot speeds
                }

                loop_count++;
                current_time = (int) SystemClock.elapsedRealtime() - start_time; //Update current time
                t_delta = (int) SystemClock.elapsedRealtime() - t_interval_start; //Update accumulation interval
            }

            try {
                app.runOnUiThread(new Runnable() {
                    @Override
                    public void run(){
                        app.debugTextView.setText(String.format(
                                "Obstacle seen"
                        ));
                    }
                });

            } catch (Exception e){
                e.printStackTrace();
            }

            Command.go( new double[] { 0, 0} );
            try{
                sleep( 1000 );
            } catch (Exception e){
                e.printStackTrace();
            }
        }

    };


    Runnable startKlinokinesis = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double current_familiarity;
            double maximum_unfamiliarity = app.model.calculateMaximumUnfamiliarity();
            LogToFileUtils.write("Maximum Unfamiliarity: " + maximum_unfamiliarity + "\n");
            double normalised_familiarity;

            // Tuning parameters
            double alpha = 80.0;
            double beta = 0.1;

            // Moving Decision
            double turn_angle;
            double step_size;

            int flag = 1;

            while(true) {
                if (app.images_access){
                    Mat matImage = app.processedDestImage;
                    //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                    byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                    matImage.get(0, 0, imageArray_tmp);
                    int[] imageArray = new int[imageArray_tmp.length];
                    for (int n = 0; n < imageArray_tmp.length; n++) {
                        imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                    }

                    current_familiarity = app.model.calculateFamiliarity(imageArray);
                    LogToFileUtils.write("Current Familiarity: " + current_familiarity+ "\n");

                    normalised_familiarity = current_familiarity / maximum_unfamiliarity;
                    LogToFileUtils.write("Normalised Familiarity: " + normalised_familiarity + "\n");

                    turn_angle = alpha * normalised_familiarity;
                    LogToFileUtils.write("Turn Angle: " + turn_angle + "\n");

                    step_size = (1 - normalised_familiarity) * beta;
                    LogToFileUtils.write("Step Size: " + step_size + "\n");


                    try {
                        Command.turnAround(flag * turn_angle);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    Command.moveForward(step_size);

                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    flag = -flag;
                }
            }
        }
    };


    Runnable startCombiner = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            SimpleMatrix tl2 = new SimpleMatrix(CX_MB.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX_MB.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX_MB.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX_MB.n_cpu4, 1);
            memory = app.stored_memory.copy();
            SimpleMatrix cpu4 = new SimpleMatrix(CX_MB.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX_MB.n_cpu1, 1);
            cpu1.set(0);

            // Zhaoyu added this
            SimpleMatrix en = new SimpleMatrix(CX_MB.n_tb1, 1);
            en.set(0);

            int T_inbound = 30;

            app.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            app.CURRiteration = 0;
            int currentTime = (int) SystemClock.elapsedRealtime() - t0;

            Boolean image_not_accessed;

            while (currentTime < T_inbound * 1000) {
                app.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                image_not_accessed = true;
                while (image_not_accessed){
                    if (app.images_access){
                        Mat matImage = app.processedDestImage;
                        //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                        byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                        matImage.get(0, 0, imageArray_tmp);
                        int[] imageArray = new int[imageArray_tmp.length];
                        for (int n = 0; n < imageArray_tmp.length; n++) {
                            imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                        }
                        en = app.cxmb.calculateFamiliarityDistribution(imageArray);
                        image_not_accessed = false;
                        LogToFileUtils.write("Image Updated");
                    }
                }

                //------   COMPASS UPDATE  -----
                tl2 = app.cxmb.tl2Output(Math.toRadians(app.currentDegree));
                cl1 = app.cxmb.cl1Output(tl2);
                tb1 = app.cxmb.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = app.cxmb.cpu4Update(memory, tb1, app.ANT_SPEED);
                cpu4 = app.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = app.cxmb.cpu1Output(tb1, cpu4, en, false);
                //cpu1 = app.cxmb.cpu1EnOutput(tb1, en);
                app.CXmotor = app.cxmb.motorOutput(cpu1);
                //writeToFile(memory, app.leftCXFlow, app.rightCXFlow, "run_1");

                // ----- ANTBOT DRIVING--------
                app.CXnewHeading = Math.toDegrees(Math.toRadians(app.currentDegree) - app.CXmotorChange * app.CXmotor);
                app.CXtheta = (app.CXnewHeading - app.currentDegree) % 360;

                t = (int) SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if (t > 300) {
                    // speed in dm/sec
                    if (app.CXtheta < -1.5) {
                        Command.go(new double[]{10, 100});
                        app.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (app.CXtheta > 1.5) {
                        Command.go(new double[]{100, 10});
                        app.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        app.ANT_SPEED = 4.0;
                        direction = "straight";
                    }
                    Log.d(app.TAG, "facing " + app.currentDegree + ", going " + direction + "\nwant to go to " + app.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(app.TAG, Util.printMemory(memory));
                }

                try {
                    app.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            app.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d" +
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f" +
                                            "\nRight Speed: %f" +
                                            "\nWant to go to: %f" +
                                            "\nSo turning of: %f",
                                    app.CURRiteration,
//                                    app.currentDegree,
                                    app.leftCXFlow,
                                    app.rightCXFlow,
                                    app.CXnewHeading,
                                    app.CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if (Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - app.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                app.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        app.debugTextView.setText(String.format("The PI Run is over! " +
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };


    Runnable startScanning = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            int min_index = 0;

            while (true) {
                try {
                    Command.turnAround(30.00);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                Boolean image_not_accessed;

                String unfarmiliarity_distribution = "";

                for (int i = 0; i < 11; i++) {
                    image_not_accessed = true;
                    while (image_not_accessed){
                        if (app.images_access){
                            Mat matImage = app.processedDestImage;
                            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                            matImage.get(0, 0, imageArray_tmp);
                            int[] imageArray = new int[imageArray_tmp.length];
                            for (int n = 0; n < imageArray_tmp.length; n++) {
                                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                            }
                            app.familarity_array[i] = app.model.calculateFamiliarity(imageArray);
                            unfarmiliarity_distribution += app.familarity_array[i] + ",";
                            image_not_accessed = false;
                        }}

                    if (i != 10) {
                        try {
                            Command.turnAround(-6.00);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                }

                LogToFileUtils.write(unfarmiliarity_distribution+'\n');

                min_index = Util.getMinIndex(app.familarity_array);

                LogToFileUtils.write("Seleted index: " + min_index + "\n");

                try {
                    Command.turnAround((10 - min_index) * 6.0);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                Command.moveForward(0.10);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }
    };


}
