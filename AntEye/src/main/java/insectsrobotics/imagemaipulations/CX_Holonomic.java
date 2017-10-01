package insectsrobotics.imagemaipulations;

import org.ejml.ops.MatrixIO;
import org.ejml.simple.SimpleMatrix;
import org.opencv.core.*;
import java.util.Random;

/**
 * Created by Luca on 04-Oct-16.
 * Code adapted from python path integrator in Stone et all
 */

public class CX_Holonomic {

    //---------------------------------------------------------------------------------------

    // ---------------   PARAMETERS  ---------------------
    static int n_tl2 = 16;
    static int n_cl1 = 16;
    static int n_tb1 = 8;
    static int n_cpu4 = 16;
    static int n_cpu1A = 14;
    static int n_cpu1B = 2;
    static int n_cpu1 = n_cpu1A + n_cpu1B;
    static double tb_tb_weight = 1.;
    static double tn_prefs_default = Math.PI/4;

    SimpleMatrix tl2_prefs_default = new SimpleMatrix(new double[][] {
    	{0.},  {-0.78539816}, {-1.57079633}, {-2.35619449},  {-3.14159265},
        {-3.92699082},  {-4.71238898},  {-5.49778714},  {0.},  {-0.78539816},
        {-1.57079633},  {-2.35619449},  {-3.14159265},  {-3.92699082},  {-4.71238898},
        {-5.49778714}
    });
    double noise_default = 0.0;  // 10% additive noise


    //--------------- TUNED PARAMETERS  ----------------------
    double cpu4_mem_gain_default = 0.005;
    double cpu4_mem_loss_default = 0.0026;  // This is tuned to keep memory constant...

    double tl2_slope_tuned = 6.8;
    double tl2_bias_tuned = 3.0;

    double cl1_slope_tuned = 3.0;
    double cl1_bias_tuned = -0.5;

    double tb1_slope_tuned = 5.0;
    double tb1_bias_tuned = 0;

    double cpu4_slope_tuned = 5.0;
    double cpu4_bias_tuned = 2.5;

    double cpu1_slope_tuned = 5.0;
    double cpu1_bias_tuned = 2.5;
    //-------------------------------------------------------------------------------



    //----------------  CLASS PARAMETERS  ------------------

    double noise;
    double tl2_slope;
    double tl2_bias;
    SimpleMatrix tl2_prefs;
    double cl1_slope;
    double cl1_bias;
    double tb1_slope;
    double tb1_bias;
    double cpu4_slope;
    double cpu4_bias;
    SimpleMatrix cpu4_mem;
    double cpu4_mem_gain;
    double cpu4_mem_loss;
    double cpu1_slope;
    double cpu1_bias;
    double tn_prefs;
    double smoothed_flow;

    //---- anatomical matrices

    SimpleMatrix W_CL1_TB1;
    SimpleMatrix W_TB1_TB1;
    SimpleMatrix W_TB1_CPU1_A;
    SimpleMatrix W_TB1_CPU1_B;
    SimpleMatrix W_TB1_CPU4;
    SimpleMatrix W_TN_CPU4;
    SimpleMatrix W_CPU4_CPU1_A;
    SimpleMatrix W_CPU4_CPU1_B;
    SimpleMatrix W_CPU1_motor_A;
    SimpleMatrix W_CPU1_motor_B;

    public CX_Holonomic(){
        // parameter initialisation
        this.noise = noise_default;
        this.tl2_slope = tl2_slope_tuned;
        this.tl2_bias = tl2_bias_tuned;
        this.tl2_prefs  = tl2_prefs_default;
        this.cl1_slope  = cl1_slope_tuned;
        this.cl1_bias  = cl1_bias_tuned;
        this.tb1_slope  = tb1_slope_tuned;
        this.tb1_bias  = tb1_bias_tuned;
        this.cpu4_slope  = cpu4_slope_tuned;
        this.cpu4_bias  = cpu4_bias_tuned;
        this.cpu4_mem  = new SimpleMatrix(new double[n_cpu4][1]);
        cpu4_mem.set(0);
        this.cpu4_mem_gain  = cpu4_mem_gain_default;
        this.cpu4_mem_loss  = cpu4_mem_loss_default;
        this.cpu1_slope  = cpu1_slope_tuned;
        this.cpu1_bias  = cpu1_bias_tuned;
        this.tn_prefs = tn_prefs_default;
        this.smoothed_flow = 0;

        // Weight matrices initialisation based on anatomy.
        SimpleMatrix tmp = SimpleMatrix.identity(n_tb1);
        this.W_CL1_TB1 = tmp.combine(
                0, tmp.numCols(), tmp
        );
		this.W_TB1_TB1 = genTbTbWeights(1.);

        this.W_TB1_CPU1_A = new SimpleMatrix(new double[][]{
                {0., 1., 0., 0., 0., 0., 0., 0.},
                {0., 0., 1., 0., 0., 0., 0., 0.},
                {0., 0., 0., 1., 0., 0., 0., 0.},
                {0., 0., 0., 0., 1., 0., 0., 0.},
                {0., 0., 0., 0., 0., 1., 0., 0.},
                {0., 0., 0., 0., 0., 0., 1., 0.},
                {0., 0., 0., 0., 0., 0., 0., 1.},
                {1., 0., 0., 0., 0., 0., 0., 0.},
                {0., 1., 0., 0., 0., 0., 0., 0.},
                {0., 0., 1., 0., 0., 0., 0., 0.},
                {0., 0., 0., 1., 0., 0., 0., 0.},
                {0., 0., 0., 0., 1., 0., 0., 0.},
                {0., 0., 0., 0., 0., 1., 0., 0.},
                {0., 0., 0., 0., 0., 0., 1., 0.}
        });
        this.W_TB1_CPU1_B = new SimpleMatrix(new double[][]{
                {0, 0, 0, 0, 0, 0, 0, 1},
                {1, 0, 0, 0, 0, 0, 0, 0}
        });
        this.W_TB1_CPU4 = tmp.combine(
                tmp.numRows(),0, tmp
        );
        this.W_TN_CPU4 = new SimpleMatrix (new double[][] {
                {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}
        }).transpose();
        this.W_CPU4_CPU1_A = new SimpleMatrix(new double[][] {
                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
                {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        });
        this.W_CPU4_CPU1_B = new SimpleMatrix(new double[][] {
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        });
        this.W_CPU1_motor_A = new SimpleMatrix(new double[][] {
                {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1}
        });
        this.W_CPU1_motor_B = new SimpleMatrix(new double[][] {
                {0, 1},
                {1, 0}
        });

        // todo: add bit of code to noisify weights
    }

    //--------------- GETTERS AND SETTERS ----------------------
    public double getNoise() {
        return noise;
    }

    public void setNoise(double noise) {
        this.noise = noise;
    }

    public double getTl2_slope() {
        return tl2_slope;
    }

    public void setTl2_slope(double tl2_slope) {
        this.tl2_slope = tl2_slope;
    }

    public double getTl2_bias() {
        return tl2_bias;
    }

    public void setTl2_bias(double tl2_bias) {
        this.tl2_bias = tl2_bias;
    }

    public SimpleMatrix getTl2_prefs() {
        return tl2_prefs;
    }

    public void setTl2_prefs(SimpleMatrix tl2_prefs) {
        this.tl2_prefs = tl2_prefs;
    }

    public double getCl1_slope() {
        return cl1_slope;
    }

    public void setCl1_slope(double cl1_slope) {
        this.cl1_slope = cl1_slope;
    }

    public double getCl1_bias() {
        return cl1_bias;
    }

    public void setCl1_bias(double cl1_bias) {
        this.cl1_bias = cl1_bias;
    }

    public double getTb1_slope() {
        return tb1_slope;
    }

    public void setTb1_slope(double tb1_slope) {
        this.tb1_slope = tb1_slope;
    }

    public double getCpu4_slope() {
        return cpu4_slope;
    }

    public void setCpu4_slope(double cpu4_slope) {
        this.cpu4_slope = cpu4_slope;
    }

    public double getTb1_bias() {
        return tb1_bias;
    }

    public void setTb1_bias(double tb1_bias) {
        this.tb1_bias = tb1_bias;
    }

    public double getCpu4_bias() {
        return cpu4_bias;
    }

    public void setCpu4_bias(double cpu4_bias) {
        this.cpu4_bias = cpu4_bias;
    }

    public SimpleMatrix getCpu4_mem() {
        return cpu4_mem;
    }

    public void setCpu4_mem(SimpleMatrix cpu4_mem) {
        this.cpu4_mem = cpu4_mem;
    }

    public double getCpu4_mem_gain() {
        return cpu4_mem_gain;
    }

    public void setCpu4_mem_gain(double cpu4_mem_gain) {
        this.cpu4_mem_gain = cpu4_mem_gain;
    }

    public double getCpu4_mem_loss() {
        return cpu4_mem_loss;
    }

    public void setCpu4_mem_loss(double cpu4_mem_loss) {
        this.cpu4_mem_loss = cpu4_mem_loss;
    }

    public double getCpu1_slope() {
        return cpu1_slope;
    }

    public void setCpu1_slope(double cpu1_slope) {
        this.cpu1_slope = cpu1_slope;
    }

    public double getCpu1_bias() {
        return cpu1_bias;
    }

    public void setCpu1_bias(double cpu1_bias) {
        this.cpu1_bias = cpu1_bias;
    }

    public SimpleMatrix getW_CL1_TB1() {
        return W_CL1_TB1;
    }

    public void setW_CL1_TB1(SimpleMatrix w_CL1_TB1) {
        W_CL1_TB1 = w_CL1_TB1;
    }

    public SimpleMatrix getW_TB1_TB1() {
        return W_TB1_TB1;
    }

    public void setW_TB1_TB1(SimpleMatrix w_TB1_TB1) {
        W_TB1_TB1 = w_TB1_TB1;
    }

    public SimpleMatrix getW_TB1_CPU1_A() {
        return W_TB1_CPU1_A;
    }

    public void setW_TB1_CPU1_A(SimpleMatrix w_TB1_CPU1_A) {
        W_TB1_CPU1_A = w_TB1_CPU1_A;
    }

    public SimpleMatrix getW_TB1_CPU1_B() {
        return W_TB1_CPU1_B;
    }

    public void setW_TB1_CPU1_B(SimpleMatrix w_TB1_CPU1_B) {
        W_TB1_CPU1_B = w_TB1_CPU1_B;
    }

    public SimpleMatrix getW_CPU4_CPU1_A() {
        return W_CPU4_CPU1_A;
    }

    public void setW_CPU4_CPU1_A(SimpleMatrix w_CPU4_CPU1_A) {
        W_CPU4_CPU1_A = w_CPU4_CPU1_A;
    }

    public SimpleMatrix getW_CPU4_CPU1_B() {
        return W_CPU4_CPU1_B;
    }

    public void setW_CPU4_CPU1_B(SimpleMatrix w_CPU4_CPU1_B) {
        W_CPU4_CPU1_B = w_CPU4_CPU1_B;
    }

    public SimpleMatrix getW_CPU1_motor_A() {
        return W_CPU1_motor_A;
    }

    public void setW_CPU1_motor_A(SimpleMatrix w_CPU1_motor_A) {
        W_CPU1_motor_A = w_CPU1_motor_A;
    }

    public SimpleMatrix getW_CPU1_motor_B() {
        return W_CPU1_motor_B;
    }

    public void setW_CPU1_motor_B(SimpleMatrix w_CPU1_motor_B) {
        W_CPU1_motor_B = w_CPU1_motor_B;
    }

    public SimpleMatrix getW_TN_CPU4() {
        return W_TN_CPU4;
    }

    public void setW_TN_CPU4(SimpleMatrix w_TN_CPU4) {
        W_TN_CPU4 = w_TN_CPU4;
    }

    //-------------------------------------------------------------------------------------
    // -------------------- private methods for internal use ------------------------------
    private SimpleMatrix genTbTbWeights (double weight) {
        SimpleMatrix W = new SimpleMatrix(n_tb1, n_tb1);
        double[] sinusoid = new double[]{-0., 0.14644661, 0.5, 0.85355339, 1., 0.85355339, 0.5, 0.14644661};
        for (int i=0; i<n_tb1; i++) {
            for (int j=0; j<n_tb1; j++) {
                int ind = (j-i)%(n_tb1);
                if (ind<0) {
                	ind = 8+ind;
                }
                W.set(i, j, sinusoid[ind]*weight);
            }
        }
        return W;
    }

    public SimpleMatrix noisySigmoid(SimpleMatrix v, double slope, double bias, double noise){
        //Takes a vector v as input, puts through sigmoid //
        for (int i=0; i<v.numRows(); i++){
            v.set(i,0,(1/(1+Math.exp(-(v.get(i,0)*slope - bias)))));
        }

        if (noise > 0) {
            Random r = new Random();
            for (int i=0; i<v.numRows(); i++){
               v.set(i,0,v.get(i,0)+r.nextGaussian()*noise);
            }
        }
        v = CX.clip(v, 0.0, 1.0);
        return v;
    }

    //--------------------------------------------------------------------------------
    //-----------public methods for class interface-----------------------------------

    public SimpleMatrix get_flow(double heading, SimpleMatrix speed) {
        if (speed.numRows() != 2) {
            if (speed.numCols() != 2) {
                throw new IllegalStateException();
            }
            else {
                speed.transpose();
            }
        }
        SimpleMatrix A = new SimpleMatrix(new double[][] {
                {Math.sin(heading - this.tn_prefs), Math.cos(heading - this.tn_prefs)},
                {Math.sin(heading + this.tn_prefs), Math.cos(heading + this.tn_prefs)}
        });
        return CX.dot(A, speed);
    }

    public SimpleMatrix tl2Output (Double theta){
        // dot product with preferred angle and current heading  	
        SimpleMatrix output = new SimpleMatrix(this.tl2_prefs.numRows(),1);
        output.set(theta);
        output = output.minus(this.tl2_prefs);
        for(int i=0; i<output.numRows(); i++){
        	output.set(i, 0, Math.cos(output.get(i)));
        }
        return noisySigmoid(output, this.tl2_slope, this.tl2_bias, this.noise);
    }

    public SimpleMatrix cl1Output (SimpleMatrix tl2){
        // Takes input from the TL2 neurons and gives output.
        return noisySigmoid(tl2.negative(), this.cl1_slope, this.cl1_bias, this.noise);
    }

    public SimpleMatrix tb1Output(SimpleMatrix cl1, SimpleMatrix tb1){
        // Ring attractor state on the protocerebral bridge.
        Double prop_cl1 = 0.667;
        Double prop_tb1 = 1.0 - prop_cl1;
        SimpleMatrix output = CX.dot(this.W_CL1_TB1, cl1).scale(prop_cl1).minus(CX.dot(this.W_TB1_TB1, tb1).scale(prop_tb1));
        return noisySigmoid(output, this.tb1_slope, this.tb1_bias, this.noise);
    }

    public SimpleMatrix tn1Output(SimpleMatrix flow) {
        SimpleMatrix ones = new SimpleMatrix(flow.numRows(), flow.numCols());
        ones.set(1);
        SimpleMatrix output = (ones.minus(flow)).divide(2.0);
        if (this.noise > 0) {
            Random r = new Random();
            for (int i=0; i<output.numRows(); i++){
                for (int j=0; j<output.numCols(); j++) {
                    output.set(i, j, output.get(i, j) + r.nextGaussian() * this.noise);
                }
            }
        }
        output = CX.clip(output, 0.0, 1.0);
        return output;
    }

    public SimpleMatrix tn2Output(SimpleMatrix flow) {
        SimpleMatrix output = flow.copy();
        if (this.noise > 0) {
            Random r = new Random();
            for (int i=0; i<output.numRows(); i++){
                for (int j=0; j<output.numCols(); j++) {
                    output.set(i, j, output.get(i, j) + r.nextGaussian() * this.noise);
                }
            }
        }
        output = CX.clip(output, 0.0, 1.0);
        return output;
    }


    public SimpleMatrix cpu4Update (SimpleMatrix cpu4_mem, SimpleMatrix tb1, SimpleMatrix tn1, SimpleMatrix tn2){
        SimpleMatrix ones = new SimpleMatrix(tb1.numRows(), tb1.numCols());
        ones.set(1);
        SimpleMatrix halfs = new SimpleMatrix(tn1.numRows(), tn1.numCols());
        halfs.set(0.5);
        //------------ compute first value to add ---------
        SimpleMatrix f1 = CX.clip(CX.dot(this.W_TN_CPU4, halfs.minus(tn1)), 0.0, 1.0);
        SimpleMatrix f2 = CX.dot(this.W_TB1_CPU4, ones.minus(tb1)).scale(this.cpu4_mem_gain);
        f1 = f1.elementMult(f2);

        // ----------- compute second value to subtract ---------
        f2 = CX.dot(this.W_TN_CPU4, tn2).scale(this.cpu4_mem_gain*0.25);

        //------------ update memory with computed values
        cpu4_mem = cpu4_mem.plus(f1);
        cpu4_mem = cpu4_mem.minus(f2);

        return CX.clip(cpu4_mem, 0.0, 1.0);
    }

    public SimpleMatrix cpu4Output (SimpleMatrix cpu4_mem){
        return noisySigmoid(cpu4_mem, this.cpu4_slope, this.cpu4_bias, this.noise);
    }

    public SimpleMatrix cpu1Output_A (SimpleMatrix tb1, SimpleMatrix cpu4){
        SimpleMatrix ones = new SimpleMatrix(tb1.numRows(), tb1.numCols());
        ones.set(1);
        SimpleMatrix inputs = CX.dot(this.W_CPU4_CPU1_A, cpu4).elementMult(CX.dot(this.W_TB1_CPU1_A, ones.minus(tb1)));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    public SimpleMatrix cpu1Output_B (SimpleMatrix tb1, SimpleMatrix cpu4){
        SimpleMatrix ones = new SimpleMatrix(tb1.numRows(), tb1.numCols());
        ones.set(1);
        SimpleMatrix inputs = CX.dot(this.W_CPU4_CPU1_B, cpu4).elementMult(CX.dot(this.W_TB1_CPU1_B, ones.minus(tb1)));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    public SimpleMatrix cpu1Output (SimpleMatrix tb1, SimpleMatrix cpu4){
        SimpleMatrix cpu1a = this.cpu1Output_A(tb1, cpu4);
        SimpleMatrix cpu1b = this.cpu1Output_B(tb1, cpu4);
        SimpleMatrix cpu1 = new SimpleMatrix(this.n_cpu1, 1);
        cpu1.set(0, 0, cpu1b.get(cpu1b.numRows()-1, 0));
        cpu1.set(cpu1.numRows()-1, 0, cpu1b.get(0, 0));
        for (int i=1; i<cpu1.numRows()-1; i++) {
        	cpu1.set(i, 0, cpu1a.get(i-1, 0));
        }
        return cpu1;
    }

    public Double motorOutput(SimpleMatrix cpu1){
        SimpleMatrix cpu1a = cpu1.extractMatrix(1, cpu1.numRows()-1, 0, 1);
        SimpleMatrix cpu1b = new SimpleMatrix(new double[][] {
                {cpu1.get(cpu1.numRows()-1, 0)},
                {cpu1.get(0, 0)}
        });
        SimpleMatrix motor = CX.dot(this.W_CPU1_motor_A, cpu1a);
        motor = motor.plus(CX.dot(this.W_CPU1_motor_B, cpu1b));
        return (motor.get(1,0) - motor.get(0,0)) * 0.25;
    }


    // METHODS FOR FLOW COMPUTATION - USE OPENCV

    public static Mat SM2Mat (SimpleMatrix M){
        Mat M2 = new Mat(M.numRows(), M.numCols(), CvType.CV_32FC1);
        for(int i=0; i<M.numRows(); i++){
            for(int j=0; j<M.numCols(); j++){
                M2.put(i,j, M.get(i, j));
            }
        }
        return M2;
    }

    public static Mat get_direction_vector(int n){
        // method which returns a matrix of row vectors containing 3D x,y,z coordinate of the
        // preferred direction vectors for optical flow
        Mat direction_matrix = new Mat(n, 3, CvType.CV_32FC1);
        double direction = -Math.PI;
        for(int i=0; i<n; i++){
            direction_matrix.put(i, 0, Math.cos(direction));
            direction_matrix.put(i, 1, Math.sin(direction));
            direction_matrix.put(i, 2, 0);
            direction += (2*Math.PI/n);
        }
        return direction_matrix;
    }

    public static Mat tn_axes(double heading){
        Mat axes = new Mat(2, 3, CvType.CV_32FC1);
        axes.put(0, 0, Math.sin(heading - Math.PI/4.0));
        axes.put(0, 1, Math.cos(heading - Math.PI/4.0));
        axes.put(0, 2, 0);
        axes.put(1, 0, Math.sin(heading + Math.PI/4.0));
        axes.put(1, 1, Math.cos(heading + Math.PI/4.0));
        axes.put(1, 2, 0);
        return axes;
    }

    public static Mat get_preferred_flow(int column_pixel_number, double heading, boolean LEFT){
        Mat axes = tn_axes(heading);
        Mat D = get_direction_vector(column_pixel_number);
//        Mat preferred_flow = new Mat(D.rows(), D.cols(), D.type());
//        Mat partial_cross_product;
//
//        int ax = 0;
//        if(!LEFT) ax = 1;
//
//        for(int i=0; i<preferred_flow.rows(); i++){
//            partial_cross_product = D.row(i).cross(axes.row(ax)).cross(D.row(i));
//            for(int j=0; j<preferred_flow.cols(); j++){
//                preferred_flow.put(i, j, partial_cross_product.get(0, j));
//            }
//        }

        Mat preferred_flow = new Mat(90, 3, CvType.CV_32FC1);
        double direction;
        direction = -Math.PI;
        for(int i=0; i<90; i++){
            preferred_flow.put(i, 0, Math.sin(direction));
            preferred_flow.put(i, 1, 0);
            preferred_flow.put(i, 2, 0);
            direction += (2*Math.PI/90);
        }
        return preferred_flow;
    }

}
