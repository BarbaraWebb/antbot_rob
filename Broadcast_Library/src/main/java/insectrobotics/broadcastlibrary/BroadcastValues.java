package insectrobotics.broadcastlibrary;


public interface BroadcastValues {

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////Applications/////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    String ANTEYE = "imagemaipulations";
    String SERIAL = "serialcommunicationapp";
    String COMBINER = "antbotcombiner";
    String INTEGRATOR = "pathintegrator";
    String VISUAL = "visualnavigationapp";

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////Service////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    String SERIAL_SERVICE = "insectsrobotics.serialcommunicationapp.SerialConnectionBackgroundServiceIntent";
    String COMBINER_SERVICE = "insectsrobotics.antbotcombiner.CombinerBackgroundServiceIntent";
    String INTEGRATOR_SERVICE = "insectsrobotics.pathintegrator.PathIntegratorBackgroundServiceIntent";
    String VISUAL_SERVICE = "insectsrobotics.visualnavigationapp.VisualNavigationBackgroundServiceIntent";

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////Broadcasts//////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    //_________________________________________Module Names_________________________________________
    String VN_MODULE = "VisualModule";
    String WILLSHAW = "WillshawModule";
    String PERFECT_MEMORY = "PerfectMemoryModule";
    String RUNDOWN ="RunDownModule";
    String TURN_TO_MIN = "TurnToMinimumModule";
    String SYSTEMATIC_SEARCH ="SystematicSearchModule";

    String PI_MODULE = "IntegratorModule";
    String TRIG_PI = "TrigonometricModule";
    String HAFERLACH = "HaferlachModule";
    String ENCODERS = "EncodersModule";
    String OPTICAL_FLOW ="OpticalFlowModule";
    String FUSED_SENSORS ="FusedSensorsModule";
    String PATH_INTEGRATOR ="PathIntegratorModule";

    String C_MODULE = "CombinerModule";
    String BACK_WITH_MB = "MushroomBodyModule";
    String EIGHT_ENS = "EightENsModule";
    String KLINOKINESIS = "KlinokinesisModule";
    //String SIMPLE_COMBINER = "SimpleCombinerModule";
    //String WEIGHTED_COMBINER = "WeightedCombinerModule";

    String NO_MODULE = "NoModule";

    //______________________________________AntEye Broadcasts_______________________________________
    String SERVER_CONNECTION_BROADCAST = "insectsrobotics.imagemaipulations.serialcommunicationapp.SERVER_CONNECTION_BROADCAST";
    String IMAGE_SERVER_BROADCAST = "insectsrobotics.imagemaipulations.serialcommunicationapp.IMAGE_SERVER_BROADCAST";
    String IMAGE_BROADCAST = "insectsrobotics.imagemaipulations.visualnavigationapp.IMAGE_BROADCAST";
    String VISUAL_MODULE_BROADCAST = "insectsrobotics.imagemaipulations.visualnavigationapp.VN_MODULE_BROADCAST";
    String INTEGRATOR_MODULE_BROADCAST = "insectsrobotics.imagemaipulations.pathintegrator.PI_MODULE_BROADCAST";
    String COMBINER_MODULE_BROADCAST = "insectsrobotics.imagemaipulations.antbotcombiner.COMBINER_MODULE_BROADCAST";
    String FLOW_BROADCAST = "insectsrobotics.imagemaipulations.antbotcombiner.FLOW_BROADCAST";
    String ANGLE_BROADCAST = "insectsrobotics.imagemaipulations.antbotcombiner.ANGLE_BROADCAST";
    String DISTANCE_BROADCAST = "insectsrobotics.imagemaipulations.antbotcombiner.DISTANCE_BROADCAST";
    String SPEED_BROADCAST = "insectsrobotics.imagemaipulations.antbotcombiner.SPEED_BROADCAST";
    String OPTICAL_FLOW_ANGLE_BROADCAST = "insectsrobotics.imagemaipulations.pathintegrator.OPTICAL_FLOW_ANGLE_BROADCAST";
    String ALERT_BROADCAST = "insectsrobotics.imagemaipulations.antbotcombiner.ALERT_BROADCAST";
    String IMAGE_ROTATE_BROADCAST = "insectsrobotics.imagemaipulations.visualnavigationapp.IMAGE_ROTATE_BROADCAST";
    String COMPLETE_FLAG_BROADCAST = "insectsrobotics.imagemaipulations.visualnavigationapp.COMPLETE_FLAG_BROADCAST";
    String START_HOME_BROADCAST = "insectsrobotics.imagemaipulations.visualnavigationapp.START_HOME_BROADCAST";
    String START_SEARCH_BROADCAST = "insectsrobotics.imagemaipulations.visualnavigationapp.START_SEARCH_BROADCAST";



    //_______________________________SerialCommunication Broadcasts_________________________________
    String WHEEL_ENCODER_BROADCAST = "insectsrobotics.serialcommunicationapp.pathintegrator.WHEEL_ENCODER_BROADCAST";
    String TASK_EXECUTED_BROADCAST = "insectsrobotics.serialcommunicationapp.antbotcombiner.TASK_EXECUTED_BROADCAST";
    String HOMING_ROUTING_BROADCAST = "insectsrobotics.serialcommunicationapp.antbotcombiner.HOMING_ROUTING_BROADCAST";
    String SERIAL_CONNECTION_ESTABLISHED_BROADCAST = "insectsrobotics.serialcommunicationapp.imagemaipulations.SERIAL_CONNECTION_ESTABLISHED_BROADCAST";
    String SERVER_CONNECTION_ESTABLISHED_BROADCAST = "insectsrobotics.serialcommunicationapp.imagemaipulations.SERVER_CONNECTION_ESTABLISHED_BROADCAST";
    String LEARN_IMAGE_COMMAND_BROADCAST = "insectsrobotics.serialcommunicationapp.antbotcombiner.LEARN_IMAGE_COMMAND_BROADCAST";
    String RESET_LEARNING_COMMAND_BROADCAST = "insectsrobotics.serialcommunicationapp.visualnavigationapp.RESET_LEARNING_COMMAND_BROADCAST";
    String CALCULATE_ERROR_COMMAND_BROADCAST = "insectsrobotics.serialcommunicationapp.antbotcombiner.CALCULATE_ERROR_COMMAND_BROADCAST";
    String TURN_TO_MINIMUM_BROADCAST = "insectsrobotics.serialcommunicationapp.antbotcombiner.TURN_TO_MINIMUM_BROADCAST";
    String TYPE_OF_MOVEMENT_BROADCAST = "insectsrobotics.serialcommunicationapp.antbotcombiner.TYPE_OF_MOVEMENT_BROADCAST";
    String SEND_STRING_BROADCAST = "insectsrobotics.serialcommunicationapp.visualnavigationapp.SEND_STRING_BROADCAST";

    //_____________________________________Combiner Broadcasts______________________________________
    String DECISION_BROADCAST = "insectsrobotics.antbotcombiner.serialcommunicationapp.DECISION_BROADCAST";
    String REQUEST_PI_DATA_BROADCAST = "insectsrobotics.antbotcombiner.pathintegrator.REQUEST_PI_DATA_BROADCAST";
    String REQUEST_VN_DATA_BROADCAST = "insectsrobotics.antbotcombiner.visualnavigationapp.REQUEST_VN_DATA_BROADCAST";
    String TURN_ANGLE_BROADCAST = "insectsrobotics.antbotcombiner.imagemaipulations.TURN_ANGLE_BROADCAST";

    //____________________________________Integrator Broadcasts_____________________________________
    String VECTOR_BROADCAST = "insectsrobotics.pathintegrator.antbotcombiner.VECTOR_BROADCAST";
    String POS_BROADCAST = "insectsrobotics.pathintegrator.imagemaipulations.POS_BROADCAST";
    String POSY_BROADCAST = "insectsrobotics.pathintegrator.imagemaipulations.POSY_BROADCAST";
    String ORI_BROADCAST = "insectsrobotics.pathintegrator.imagemaipulations.ORI_BROADCAST";
    String PI_BROADCAST = "insectsrobotics.pathintegrator.antbotcombiner.PI_BROADCAST";
    String DISTANCE_DATA_BROADCAST = "insectsrobotics.pathintegrator.antbotcombiner.DISTANCE_DATA_BROADCAST";
    String ORIENTATION_DATA_BROADCAST = "insectsrobotics.pathintegrator.antbotcombiner.ORIENTATION_DATA_BROADCAST";



    //_________________________________VisualNavigation Broadcasts__________________________________
    String NUMBER_OF_IMAGES_BROADCAST = "insectsrobotics.visualnavigationapp.imagemaipulations.NUMBER_OF_IMAGES_BROADCAST";
    String STATUS_UPDATE_BROADCAST = "insectsrobotics.visualnavigationapp.antbotcombiner.STATUS_UPDATE_BROADCAST";
    String ERROR_BROADCAST = "insectsrobotics.visualnavigationapp.antbotcombiner.ERROR_BROADCAST";
    String MOVEMENT_DECISION_BROADCAST = "insectsrobotics.visualnavigationapp.antbotcombiner.MOVEMENT_DECISION_BROADCAST";
    String TURN_DECISION_BROADCAST = "insectsrobotics.visualnavigationapp.antbotcombiner.TURN_DECISION_BROADCAST";


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////Actions///////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //______________________________________AntEye Broadcasts_______________________________________
    String SERVER_CONNECTION = "SERVER_CONNECTION_BROADCAST";
    String IMAGE_SERVER = "IMAGE_SERVER_BROADCAST";
    String IMAGE = "IMAGE_BROADCAST";
    String VISUAL_MODULE = "VN_MODULE_BROADCAST";
    String INTEGRATOR_MODULE = "PI_MODULE_BROADCAST";
    String COMBINER_MODULE = "COMBINER_MODULE_BROADCAST";
    String COMBINER_FLOW = "FLOW_BROADCAST";
    String COMBINER_ANGLE = "ANGLE_BROADCAST";
    String COMBINER_DISTANCE = "DISTANCE_BROADCAST";
    String COMBINER_SPEED = "SPEED_BROADCAST";
    String INTEGRATOR_FLOW = "OPTICAL_FLOW_ANGLE_BROADCAST";
    String ALERT = "ALERT_BROADCAST";
    String IMAGE_ROTATE ="IMAGE_ROTATE_BROADCAST";
    String COMPLETE_FLAG ="COMPLETE_FLAG_BROADCAST";
    String START_HOME ="START_HOME_BROADCAST";
    String START_SEARCH ="START_SEARCH_BROADCAST";


    //_______________________________SerialCommunication Broadcasts_________________________________
    String WHEEL_ENCODER = "WHEEL_ENCODER_BROADCAST";
    String TASK_EXECUTED = "TASK_EXECUTED_BROADCAST";
    String HOMING_ROUTING = "HOMING_ROUTING_BROADCAST";
    String SERIAL_CONNECTION_ESTABLISHED = "SERIAL_CONNECTION_ESTABLISHED_BROADCAST";
    String SERVER_CONNECTION_ESTABLISHED = "SERVER_CONNECTION_ESTABLISHED_BROADCAST";
    String LEARN_IMAGE_COMMAND = "LEARN_IMAGE_COMMAND_BROADCAST";
    String CALCULATE_ERROR_COMMAND = "CALCULATE_ERROR_COMMAND_BROADCAST";
    String RESET_LEARNING_COMMAND = "RESET_LEARNING_COMMAND_BROADCAST";
    String TURN_TO_MINIMUM = "TURN_TO_MINIMUM_BROADCAST";
    String TYPE_OF_MOVEMENT = "TYPE_OF_MOVEMENT_BROADCAST";
    String SEND_STRING = "SEND_STRING_BROADCAST";

    //_____________________________________Combiner Broadcasts______________________________________
    String DECISION = "DECISION_BROADCAST";
    String REQUEST_PI_DATA = "REQUEST_PI_DATA_BROADCAST";
    String REQUEST_VN_DATA = "REQUEST_VN_DATA_BROADCAST";
    String TURN_AE_DATA = "TURN_ANGLE_BROADCAST";

    //____________________________________Integrator Broadcasts_____________________________________
    String VECTOR = "VECTOR_BROADCAST";
    String POS = "POS_BROADCAST";
    String POSY = "POSY_BROADCAST";
    String ORI = "ORI_BROADCAST";
    String PI="PI_BROADCAST";
    String DIST_DATA="DISTANCE_DATA_BROADCAST";
    String ORI_DATA ="ORIENTATION_DATA_BROADCAST";


    //_________________________________VisualNavigation Broadcasts__________________________________
    String NUMBER_OF_IMAGES = "NUMBER_OF_IMAGES_BROADCAST";
    String STATUS_UPDATE = "STATUS_UPDATE_BROADCAST";
    String ERROR = "ERROR_BROADCAST";
    String MOVEMENT_DECISION = "MOVEMENT_DECISION_BROADCAST";
    String TURN_DECISION = "TURN_DECISION_BROADCAST";
    //--------------------------------Image Processing Request Codes--------------------------------
    int SAVE_IMAGE = 1;
    int SAVE_LEARN_IMAGE = 2;
    int LOAD_IMAGE = 3;
    int LOAD_LEARNED_IMAGE = 4;
    int LEARN_IMAGE = 5;
    int CALCULATE_ERROR = 6;
    int SETUP_ALGORITHM = 7;
    //---------------------------------Image Processing Error Codes---------------------------------

}
