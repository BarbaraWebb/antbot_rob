package insectsrobotics.imagemaipulations;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;

/**
 * Created by Luca on 04-Oct-16.
 * Code adapted from python path integrator in Stone et all
 */

public class CX {

    //---------------------------------------------------------------------------------------

    // ---------------   PARAMETERS  ---------------------
    static int n_tl2 = 16;
    static int n_cl1 = 16;
    static int n_tb1 = 8;
    static int n_cpu4 = 16;
    static int n_cpu1 = 16;
    static double tb_tb_weight = 1.;
    SimpleMatrix tl2_prefs_default = new SimpleMatrix(new double[][] { 
    	{0.},  {0.78539816}, {1.57079633}, {2.35619449},  {3.14159265},
        {3.92699082},  {4.71238898},  {5.49778714},  {0.},  {0.78539816},
        {1.57079633},  {2.35619449},  {3.14159265},  {3.92699082},  {4.71238898},
        {5.49778714}
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

    double cpu1_slope_tuned = 6.0;
    double cpu1_bias_tuned = 2.0;
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

    //---- anatomical matrices

    SimpleMatrix W_CL1_TB1;
    SimpleMatrix W_TB1_TB1;
    SimpleMatrix W_TB1_CPU1;
    SimpleMatrix W_TB1_CPU4;
    SimpleMatrix W_CPU4_CPU1;
    SimpleMatrix W_CPU1_motor;

    public CX(){
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

        // Weight matrices initialisation based on anatomy.
        SimpleMatrix tmp = SimpleMatrix.identity(n_tb1);
        this.W_CL1_TB1 = tmp.combine(
                0, tmp.numCols(), tmp
        );
		this.W_TB1_TB1 = genTbTbWeights(1.);
        this.W_TB1_CPU1 = tmp.combine(
                tmp.numRows(),0, tmp
        );
        this.W_TB1_CPU4 = tmp.combine(
                tmp.numRows(),0, tmp
        );

        this.W_CPU4_CPU1 = new SimpleMatrix(new double[][] {
        {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
        {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
        {0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0}
        }).transpose();
        this.W_CPU1_motor = new SimpleMatrix(new double[][] {{-1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1}});
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

    public SimpleMatrix getW_TB1_CPU1() {
        return W_TB1_CPU1;
    }

    public void setW_TB1_CPU1(SimpleMatrix w_TB1_CPU1) {
        W_TB1_CPU1 = w_TB1_CPU1;
    }

    public SimpleMatrix getW_TB1_CPU4() {
        return W_TB1_CPU4;
    }

    public void setW_TB1_CPU4(SimpleMatrix w_TB1_CPU4) {
        W_TB1_CPU4 = w_TB1_CPU4;
    }

    public SimpleMatrix getW_CPU4_CPU1() {
        return W_CPU4_CPU1;
    }

    public void setW_CPU4_CPU1(SimpleMatrix w_CPU4_CPU1) {
        W_CPU4_CPU1 = w_CPU4_CPU1;
    }

    public SimpleMatrix getW_CPU1_motor() {
        return W_CPU1_motor;
    }

    public void setW_CPU1_motor(SimpleMatrix w_CPU1_motor) {
        W_CPU1_motor = w_CPU1_motor;
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

    public static SimpleMatrix noisySigmoid(SimpleMatrix v, double slope, double bias, double noise){
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
        for (int i=0; i<v.numRows(); i++){
            if (v.get(i,0) > 1){
                v.set(i,0,1);
            } else if (v.get(i,0) < 0){
                v.set(i,0,0);
            }
        }
        return v;
    }

    //--------------------------------------------------------------------------------
    //-----------public methods for class interface-----------------------------------

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

    public SimpleMatrix cpu4Update (SimpleMatrix cpu4_mem, SimpleMatrix tb1, Double speed){
        SimpleMatrix ones = new SimpleMatrix(tb1.numRows(), 1);
        ones.set(1);
        SimpleMatrix diffMatrix = new SimpleMatrix(cpu4_mem.numRows(), cpu4_mem.numCols());
        diffMatrix.set(speed*this.cpu4_mem_loss);
        cpu4_mem = cpu4_mem.minus(diffMatrix);
        diffMatrix.set(CX.dot(this.W_TB1_CPU4, ones.minus(tb1)).scale(speed*this.cpu4_mem_gain));
        cpu4_mem = cpu4_mem.plus(diffMatrix);
        for (int i=0; i<cpu4_mem.numRows(); i++){
            if (cpu4_mem.get(i,0) > 1){
                cpu4_mem.set(i,0,1);
            } else if (cpu4_mem.get(i,0) < 0){
                cpu4_mem.set(i,0,0);
            }
        }
        return cpu4_mem;
    }

    public SimpleMatrix cpu4Output (SimpleMatrix cpu4_mem){
        return noisySigmoid(cpu4_mem, this.cpu4_slope, this.cpu4_bias, this.noise);
    }

    public SimpleMatrix cpu1Output (SimpleMatrix tb1, SimpleMatrix cpu4){
        SimpleMatrix inputs = CX.dot(this.W_CPU4_CPU1, cpu4).minus((CX.dot(this.W_TB1_CPU1, tb1)));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    public Double motorOutput(SimpleMatrix cpu1){
        return this.W_CPU1_motor.dot(cpu1);
    }


    //------------------------- Helper custom Matrix methods --------------------------
    //---------------------------------------------------------------------------------
    public static SimpleMatrix insertInColumn(SimpleMatrix M, SimpleMatrix c, Integer index){
        if (c.numCols() > 1){
            if(c.numRows() == 1){
                c.transpose();
            } else {
                throw new IllegalStateException();
            }
        }

        for (int i=0; i<c.numRows(); i++){
            M.set(i, index, c.get(i,0));
        }
        return M;
    }

    public static SimpleMatrix insertInRow(SimpleMatrix M, SimpleMatrix c, Integer index){
        return insertInColumn(M.transpose(), c.transpose(), index).transpose();
    }
    
    public static SimpleMatrix dot(SimpleMatrix M, SimpleMatrix T){
    	SimpleMatrix R = new SimpleMatrix(M.numRows(), T.numCols());
    	for(int i=0; i<M.numRows(); i++){
    		for(int j=0; j<T.numCols(); j++){
        		R.set(i,j,M.extractVector(true, i).dot(T.extractVector(false, j)));
    		}
    	}
		return R;
    }

    public static SimpleMatrix clip(SimpleMatrix M, double lower_bound, double upper_bound) {
        for (int i=0; i<M.numRows(); i++){
            for (int j=0; j<M.numCols(); j++) {
                if (M.get(i,j) < lower_bound) {
                    M.set(i,j, lower_bound);
                } else if (M.get(i,j) > upper_bound) {
                    M.set(i,j, upper_bound);
                }
            }
        }
        return M;
    }
}
