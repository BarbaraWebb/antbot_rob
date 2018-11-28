package insectsrobotics.imagemaipulations.ThreadArchive;

import insectsrobotics.imagemaipulations.*;

import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;

import static java.lang.Thread.sleep;

import android.os.SystemClock;
import android.util.Log;

public class CentralComplexThreads {
    private MainActivity main = null;
    //
    // Constructor. For legacy reasons we need access to MainActivity for various global variables
    //
    public CentralComplexThreads(MainActivity main){
        this.main = main;
    }

    public Runnable CXthread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Log.i(flowMainActivity.TAG, "" + xPosOF + "," + yPosOF+","+orientation)
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

            main.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            main.CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-main.startTime;
            while (currentTime<T_outbound*1000) {
                main.CURRiteration++;
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format("outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\ncurrent iteration: %d"+
                                            "\ncurrent direction: %f",
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.CURRiteration,
                                    main.currentDegree));
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
                        main.ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        Command.go(new double[]{10, 100});
                        main.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        Command.go(new double[]{100, 100});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    // based on odometry
                    //main.ANT_SPEED = ()

                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction);
                    Log.d(MainActivity.TAG, Util.printMemory(memory));
                }
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, main.ANT_SPEED);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                main.CXmotor = c.motorOutput(cpu1);
                Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run_1", main.frame_rate_cx, main.CURRiteration, main.CXnewHeading);
                currentTime = (int) SystemClock.elapsedRealtime()-main.startTime;
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

            main.startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            Boolean algorithm_not_setted = true;

            //
            // Image storage lock mechanism
            //
            while (algorithm_not_setted){
                if (main.images_access){
                    main.new_image = main.new SaveImages();
                    main.new_image.execute(main.processedDestImage, 7);
                    algorithm_not_setted = false;
                }
            }

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){
                main.CURRiteration++;
                try {
                    sleep(900);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if (main.images_access) {
                    main.new_image = main.new SaveImages();
                    main.new_image.execute(main.processedDestImage, 5);
                }

                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, main.ANT_SPEED);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                main.CXmotor = c.motorOutput(cpu1);
                Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run_1",main.frame_rate_cx, main.CURRiteration, main.CXnewHeading);

                // ----- ANTBOT DRIVING--------
                main.CXnewHeading = Math.toDegrees(Math.toRadians(main.currentDegree) - main.CXmotorChange * main.CXmotor);
                main.CXtheta = (main.CXnewHeading - main.currentDegree)%360;

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if(t > 300){
                    // speed in dm/sec
                    if (main.CXtheta<-1.5){
                        Command.go(new double[]{10, 100});
                        main.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (main.CXtheta>1.5){
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction + "\nwant to go to " + main.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(MainActivity.TAG, Util.printMemory(memory));
                }

                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d"+
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\nWant to go to: %f"+
                                            "\nSo turning of: %f",
                                    main.CURRiteration,
//                                    main.currentDegree,
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.CXnewHeading,
                                    main.CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - main.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                main.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        main.debugTextView.setText(String.format("The PI Run is over! "+
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));

                        // Show the "Put Robot Back to Food" Alert Dialog!
                        main.showDialog();
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    public Runnable CXMBthread = new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Log.i(flowMainActivity.TAG, "" + xPosOF + "," + yPosOF+","+orientation)

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

            main.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            main.CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-main.startTime;
            while (currentTime<T_outbound*1000) {
                main.CURRiteration++;
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format("outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\ncurrent iteration: %d"+
                                            "\ncurrent direction: %f",
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.CURRiteration,
                                    main.currentDegree));
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
                        main.ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        Command.go(new double[]{10, 100});
                        main.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        Command.go(new double[]{100, 100});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    // based on odometry
                    //main.ANT_SPEED = ()

                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction);
                    Log.d(MainActivity.TAG, Util.printMemory(memory));
                }
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = main.cxmb.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = main.cxmb.cl1Output(tl2);
                tb1 = main.cxmb.tb1Output(cl1, tb1);

//                String tb1_str = "";
//                for(int i = 0; i < 8; i++){
//                    tb1_str += tb1.get(i, 0) + " ";
//                }
//                LogToFileUtils.write(tb1_str);

                // ------ DISPLACEMENT UPDATE -----
                memory = main.cxmb.cpu4Update(memory, tb1, main.ANT_SPEED);
                cpu4 = main.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = main.cxmb.cpu1Output(tb1, cpu4, en, true);
                main.CXmotor = main.cxmb.motorOutput(cpu1);
                Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run_1", main.frame_rate_cx, main.CURRiteration, main.CXnewHeading);
                currentTime = (int) SystemClock.elapsedRealtime()-main.startTime;
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

            main.startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            main.stored_memory = memory.copy();

            Boolean algorithm_not_setted = true;

            while (algorithm_not_setted){
                if (main.images_access){
                    main.new_image = main.new LearnDirectionImages();
                    main.new_image.execute(main.processedDestImage, 7);
                    algorithm_not_setted = false;
                }
            }

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){
                main.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //------   COMPASS UPDATE  -----
                tl2 = main.cxmb.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = main.cxmb.cl1Output(tl2);
                tb1 = main.cxmb.tb1Output(cl1, tb1);

                if (main.images_access) {
                    main.new_image = main.new LearnDirectionImages();
                    main.new_image.execute(main.processedDestImage, 5, tb1);
                }

                // ------ DISPLACEMENT UPDATE -----
                memory = main.cxmb.cpu4Update(memory, tb1, main.ANT_SPEED);
                cpu4 = main.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = main.cxmb.cpu1Output(tb1, cpu4, en, true);
                main.CXmotor = main.cxmb.motorOutput(cpu1);
                Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run_1",main.frame_rate_cx, main.CURRiteration, main.CXnewHeading);

                // ----- ANTBOT DRIVING--------
                main.CXnewHeading = Math.toDegrees(Math.toRadians(main.currentDegree) - main.CXmotorChange * main.CXmotor);
                main.CXtheta = (main.CXnewHeading - main.currentDegree)%360;

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if(t > 300){
                    // speed in dm/sec
                    if (main.CXtheta<-1.5){
                        Command.go(new double[]{10, 100});
                        main.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (main.CXtheta>1.5){
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction + "\nwant to go to " + main.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(MainActivity.TAG, Util.printMemory(memory));
                }

                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d"+
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f"+
                                            "\nWant to go to: %f"+
                                            "\nSo turning of: %f",
                                    main.CURRiteration,
//                                    main.currentDegree,
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.CXnewHeading,
                                    main.CXtheta));
                            //Toast.makeText(getApplication(), "famialarity:" + familarity, Toast.LENGTH_SHORT);
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - main.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                main.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        main.debugTextView.setText(String.format("The PI Run is over! "+
                                "\nPlease Push the \"yes\" button when you are ready."));
                        // Show the "Put Robot Back to Food" Alert Dialog!
                        main.showDialog();
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    //
    // Modified by RM, outbound time reduced to 25 seconds and speeds adjusted for new power supply
    //
    public Runnable CXHolonomicThread=new Runnable() {
        @Override
        public void run() {
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            CX_H_Pontin c = new CX_H_Pontin();
            Integer T_outbound = 25;            // outbound for 40 sec
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
            main.CURRiteration = 0;
            int currentTime = (int)SystemClock.elapsedRealtime()-startTime;

            while (currentTime<T_outbound*1000) {
                main.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // print output to screen - useful for DEBUG
                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format(
                                    "outbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f" +
                                            "\nFrameRate: %f",
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.frame_rate_cx
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
                        Command.go(new double[]{15, 15});
                        main.ANT_SPEED = 4.0;
                        direction = "straight";
                    } else if (currentTime < 8000) {
                        Command.go(new double[]{5, 15});
                        main.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 11000) {
                        Command.go(new double[]{15, 5});
                        main.ANT_SPEED = 2.0;
                        direction = "left";
                    } else if (currentTime < 25000) {
                        Command.go(new double[]{15, 15});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    } else {
                        Command.go(new double[]{15, 5});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                }

                t0 = (int) SystemClock.elapsedRealtime();
                Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction);
                Log.d(MainActivity.TAG, Util.printMemory(memory));
                // updates in memory structure as we move which will allow return to nest
                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = c.cl1Output(tl2);
                tb1 = c.tb1Output(cl1, tb1);

                //---- Speed retrieval ------------
                SimpleMatrix flow = new SimpleMatrix(new double[][]{{main.leftCXFlow},{main.rightCXFlow}});
                SimpleMatrix tn1 = c.tn1Output(flow);
                SimpleMatrix tn2 = c.tn2Output(flow);

                // ------ DISPLACEMENT UPDATE -----
                memory = c.cpu4Update(memory, tb1, tn1, tn2);
                cpu4 = c.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = c.cpu1Output(tb1, cpu4);
                main.CXmotor = c.motorOutput(cpu1);
                currentTime = (int) SystemClock.elapsedRealtime()-startTime;
                Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run",main.frame_rate_cx, main.CURRiteration, main.CXnewHeading);
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

            startTime = (int) SystemClock.elapsedRealtime();
            t0 = (int)SystemClock.elapsedRealtime();
            t = 0;

            // INBOUND ROUTE
            while (currentTime < T_inbound*1000){

                main.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //------   COMPASS UPDATE  -----
                tl2 = c.tl2Output(Math.toRadians(main.currentDegree));
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
                main.CXmotor = c.motorOutput(cpu1);
                Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow,  "run",main.frame_rate_cx, main.CURRiteration, main.CXnewHeading);

                // ----- ANTBOT DRIVING--------

                // new direction based on memory
                main.CXnewHeading = Math.toDegrees(
                        (Math.toRadians(main.currentDegree) + main.CXmotor + Math.PI)
                                %(2.0 * Math.PI) - Math.PI
                );

                // given new direction, turn a litte right/left to approach it
                main.CXtheta = ((main.CXnewHeading - main.currentDegree) % 360);

                t = (int)SystemClock.elapsedRealtime() - t0;
                String direction = "";
                if(t > 300){
                    // speed in dm/sec
                    if (main.CXtheta < -1.5){
                        Command.go(new double[]{5, 15});
                        main.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (main.CXtheta>1.5){
                        Command.go(new double[]{15, 5});
                        main.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{15, 15});
                        main.ANT_SPEED = 4.;
                        direction = "straight";
                    }
                    Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction + "\nwant to go to " + main.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(MainActivity.TAG, Util.printMemory(memory));
                }

                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            float frame_rate;
                            if (main.currentFrameTime-main.prevFrameTime != 0){
                                frame_rate = 1/((main.currentFrameTime-main.prevFrameTime)*1000);
                            } else {
                                frame_rate = 0;
                            }
                            main.debugTextView.setText(String.format(
                                    "inbound " +
                                            "\nLeft Speed: %f"+
                                            "\nRight Speed: %f" +
                                            "\nFrameRate: %f",
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    frame_rate
                                    )
                            );
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                currentTime = (int) SystemClock.elapsedRealtime() - startTime;
                if(Util.isHome(memory)) break;
            }

            Command.go(new double[]{0, 0});
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                main.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        main.debugTextView.setText(String.format("The Run is over! "+
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
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
            memory = main.stored_memory.copy();
            SimpleMatrix cpu4 = new SimpleMatrix(CX_MB.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX_MB.n_cpu1, 1);
            cpu1.set(0);

            // Zhaoyu added this
            SimpleMatrix en = new SimpleMatrix(CX_MB.n_tb1, 1);
            en.set(0);

            int T_inbound = 30;

            main.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            main.CURRiteration = 0;
            int currentTime = (int) SystemClock.elapsedRealtime() - t0;

            Boolean image_not_accessed;

            while (currentTime < T_inbound * 1000) {
                main.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                image_not_accessed = true;
                while (image_not_accessed){
                    if (main.images_access){
                        Mat matImage = main.processedDestImage;
                        //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                        byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                        matImage.get(0, 0, imageArray_tmp);
                        int[] imageArray = new int[imageArray_tmp.length];
                        for (int n = 0; n < imageArray_tmp.length; n++) {
                            imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                        }
                        en = main.cxmb.calculateFamiliarityDistribution(imageArray);
                        image_not_accessed = false;
                        LogToFileUtils.write("Image Updated");
                    }
                }

                //------   COMPASS UPDATE  -----
                tl2 = main.cxmb.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = main.cxmb.cl1Output(tl2);
                tb1 = main.cxmb.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = main.cxmb.cpu4Update(memory, tb1, main.ANT_SPEED);
                cpu4 = main.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = main.cxmb.cpu1Output(tb1, cpu4, en, false);
                //cpu1 = main.cxmb.cpu1EnOutput(tb1, en);
                main.CXmotor = main.cxmb.motorOutput(cpu1);
                //Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run_1");

                // ----- ANTBOT DRIVING--------
                main.CXnewHeading = Math.toDegrees(Math.toRadians(main.currentDegree) - main.CXmotorChange * main.CXmotor);
                main.CXtheta = (main.CXnewHeading - main.currentDegree) % 360;

                t = (int) SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if (t > 300) {
                    // speed in dm/sec
                    if (main.CXtheta < -1.5) {
                        Command.go(new double[]{10, 100});
                        main.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (main.CXtheta > 1.5) {
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        main.ANT_SPEED = 4.0;
                        direction = "straight";
                    }
                    Log.d(MainActivity.TAG, "facing " + main.currentDegree + ", going " + direction + "\nwant to go to " + main.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(MainActivity.TAG, Util.printMemory(memory));
                }

                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d" +
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f" +
                                            "\nRight Speed: %f" +
                                            "\nWant to go to: %f" +
                                            "\nSo turning of: %f",
                                    main.CURRiteration,
//                                    main.currentDegree,
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.CXnewHeading,
                                    main.CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if (Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - main.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                main.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        main.debugTextView.setText(String.format("The PI Run is over! " +
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };
}
