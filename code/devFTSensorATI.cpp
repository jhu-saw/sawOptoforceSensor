
// This is the main DLL file.

/*modifications
july.22.2005 - Sam Skuce(ATI Industrial Automation) - added support for user setting connection mode
aug.5.2005a - ss - added GetHardwareTempComp
*/

//September 2008 - removed most .NET references (marcin)



#include "devFTSensorATI.h"

#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction.h>

CMN_IMPLEMENT_SERVICES(devFTSensorATI);

#define WHOLE_LOTTA_CHARACTERS 512
#define FIRST_TORQUE_INDEX 3 /*the index of the first torque reading in 
the standard output list order (fx, fy, fz, tx, ty, tz)*/
#define NUM_FT_AXES 6		/*the number of force/torque axes*/
#define NUM_STRAIN_GAUGES 6 /*the number of strain gauges*/
#define NUM_MATRIX_ELEMENTS 36		/*the total number of elements in a calibration matrix*/
#define GAUGE_SATURATION_LEVEL 0.995 /*gauges are considered saturated when they reach 99.5% of their maximum load*/

devFTSensorATI::devFTSensorATI() :
m_hiHardware( new ATIDAQHardwareInterface ), m_Calibration ( NULL ),
m_iMaxVoltage( 10 ), m_iMinVoltage( -10 )
{	
    m_dUpperSaturationVoltage = m_iMaxVoltage * GAUGE_SATURATION_LEVEL;
    m_dLowerSaturationVoltage = m_iMinVoltage * GAUGE_SATURATION_LEVEL;
    ////xmlCalibrationFileHandler=NULL;
}

devFTSensorATI::~devFTSensorATI()
{
    //if (xmlCalibrationFileHandler)
    //    delete xmlCalibrationFileHandler; //this destroys the calibration file
    ////DAQFTCLIBRARY::destroyCalibration( m_Calibration);

    if( NULL != m_Calibration )
        DAQFTCLIBRARY::destroyCalibration( m_Calibration );
}

int devFTSensorATI::LoadCalibrationFile(const std::string &calFile )
{
    /*first, unload any currently loaded calibration*/
    if( NULL != m_Calibration )
        DAQFTCLIBRARY::destroyCalibration( m_Calibration );

    //xmlCalibrationFileHandler= new xml_calfile_handler(calFile);
    //xml_parser::kernel_1a_c parser;

    //// now associate the handlers with the parser and tell it to parse
    //parser.add_document_handler(*xmlCalibrationFileHandler);
    //parser.add_error_handler(xmlErrorHandler);
    ////do the work
    //parser.parse(xmlCalibrationFileHandler->calFile);

    ////    CMN_LOG_CLASS_INIT_VERBOSE<<xmlCalibrationFileHandler<<std::endl;

    //xmlCalibrationFileHandler->ToStream(std::cout);

    //create calibration file...
    if ( ! ParseCalibrationFile(calFile) ) {
        CMN_LOG_CLASS_INIT_ERROR<<" FAILED to LOAD CALIBRATION "<<calFile<<std::endl;
    }
    //CMN_LOG_CLASS_INIT_VERBOSE<<this->c<<std::endl;

    std::cout<<std::endl<<std::endl;
   // this->ToStream(std::cout);

   // osaSleep(50);

    //m_Calibration = xmlCalibrationFileHandler->GetCalibration();  

    if ( NULL == m_Calibration )
    {
        m_sErrorInfo = std::string( "Could not load calibration file successfully" );
        return -1;
    }	
    DAQFTCLIBRARY::ResetDefaults(m_Calibration);  //need this in order to set the working matrix etc.

    /*determine max and min voltages and saturation levels*/
    m_iMinVoltage = 0;
    m_iMaxVoltage = m_Calibration->VoltageRange;
    if ( m_Calibration->BiPolar )
    {
        m_iMinVoltage -= ( m_Calibration->VoltageRange / 2 );
        m_dLowerSaturationVoltage = m_iMinVoltage * GAUGE_SATURATION_LEVEL;
        m_iMaxVoltage -= ( m_Calibration->VoltageRange / 2 );
        m_dUpperSaturationVoltage = m_iMaxVoltage * GAUGE_SATURATION_LEVEL;
    }

    return 0;
}

bool devFTSensorATI::ParseCalibrationFile(const std::string &calFile){

    CMN_LOG_CLASS_INIT_WARNING<< "THIS PARSING METHOD MIGHT NOT WORK WITH ALL SENSORS!!!"<< std::endl;


    if( NULL != m_Calibration )
        DAQFTCLIBRARY::destroyCalibration( m_Calibration );

    //parse the file

    //allocate :

    m_Calibration = (Calibration *) calloc(1,sizeof(Calibration));

    /// initialize temp comp variables
    m_Calibration->TempCompAvailable=FALSE;
    for (int i = 0 ; i < MAX_GAUGES; i++) {
        m_Calibration->rt.bias_slopes[i]=0;
        m_Calibration->rt.gain_slopes[i]=0;
    }
    m_Calibration->rt.thermistor=0;

    //some other non required variable settings.
    m_Calibration->BasicTransform.AngleUnits=DAQFTCLIBRARY::ATI_strdup("deg\0");
    m_Calibration->cfg.UserTransform.AngleUnits=DAQFTCLIBRARY::ATI_strdup("deg\0");
    m_Calibration->BiPolar = TRUE;
    m_Calibration->VoltageRange = 20;
    m_Calibration->HWTempComp = FALSE;

    CMN_LOG_CLASS_INIT_VERBOSE << "Parsing " << calFile << std::endl;

    cmnXMLPath config;
    config.SetInputSource(calFile);

    std::string serial;
    std::string bodyStyle;
    std::string family;
    int nch = 0;

    if (!config.GetXMLValue("/FTSensor", "@Serial",     serial) || 
        !config.GetXMLValue("/FTSensor", "@BodyStyle",  bodyStyle) ||
        !config.GetXMLValue("/FTSensor", "@NumGages",   nch) ||
        !config.GetXMLValue("/FTSensor", "@Family",     family) )
    {

        CMN_LOG_CLASS_INIT_ERROR << "failed to load config " << std::endl;
        return false;
    }
    else {
        m_Calibration->rt.NumChannels = (unsigned short) nch;
        m_Calibration->Serial=DAQFTCLIBRARY::ATI_strdup(serial.c_str()); 
        m_Calibration->BodyStyle=DAQFTCLIBRARY::ATI_strdup(bodyStyle.c_str()); 
        m_Calibration->Family=DAQFTCLIBRARY::ATI_strdup(family.c_str()); 
        m_Calibration->rt.NumChannels = m_Calibration->rt.NumChannels + 1 ; //temperature compensation
    }

    std::string partNumber;
    std::string calDate;
    std::string forceUnits;
    std::string torqueUnits;
    std::string distUnits;
    std::string angleUnits;

    std::string BiPolar;
    std::string HWTempComp;

    if (!config.GetXMLValue("/FTSensor/Calibration", "@PartNumber",     partNumber) || 
        !config.GetXMLValue("/FTSensor/Calibration", "@CalDate",        calDate) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@ForceUnits",     forceUnits) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@TorqueUnits",    torqueUnits) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@DistUnits",      distUnits) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@OutputBipolar",  BiPolar) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@OutputRange",    m_Calibration->VoltageRange) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@HWTempComp",     HWTempComp))

    {
        CMN_LOG_CLASS_INIT_ERROR << "failed to load config " << std::endl;
        return false;
    }	
    else {
        //might not be available 
        if (config.GetXMLValue("/FTSensor/Calibration", "@AngleUnits",     angleUnits) ) {
            CMN_LOG_CLASS_INIT_WARNING << "Did not expect AngleUnits " << std::endl;
            m_Calibration->BasicTransform.AngleUnits=DAQFTCLIBRARY::ATI_strdup(angleUnits.c_str()); 
        }
        // set output units to calibration file defaults			
        m_Calibration->PartNumber=DAQFTCLIBRARY::ATI_strdup(partNumber.c_str()); 
        m_Calibration->CalDate=DAQFTCLIBRARY::ATI_strdup(calDate.c_str()); 
        m_Calibration->ForceUnits=DAQFTCLIBRARY::ATI_strdup(forceUnits.c_str()); 
        m_Calibration->TorqueUnits=DAQFTCLIBRARY::ATI_strdup(torqueUnits.c_str()); 
        m_Calibration->BasicTransform.DistUnits=DAQFTCLIBRARY::ATI_strdup(distUnits.c_str()); 

        if (BiPolar.compare("True")  == 0 ) {
            m_Calibration->BiPolar = 1;
        }
        else
            m_Calibration->BiPolar = 0;

        if (HWTempComp.compare("True")  == 0 ){
            m_Calibration->HWTempComp = 1;
        }
        else
            m_Calibration->HWTempComp = 0;

        m_Calibration->cfg.ForceUnits=DAQFTCLIBRARY::ATI_strdup(m_Calibration->ForceUnits); 
        m_Calibration->cfg.TorqueUnits=DAQFTCLIBRARY::ATI_strdup(m_Calibration->TorqueUnits);  // set output units to calibration file defaults
        m_Calibration->cfg.UserTransform.DistUnits=DAQFTCLIBRARY::ATI_strdup( m_Calibration->BasicTransform.DistUnits);
        m_Calibration->cfg.UserTransform.AngleUnits=DAQFTCLIBRARY::ATI_strdup(m_Calibration->BasicTransform.AngleUnits);
    }

    std::vector<double> TT;
    TT.resize(6);
 
    for (int i =0;i<6;i++){
        TT[i]=0;
    }
    if (!config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dx",    TT[0]) || 
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dy",    TT[1]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dz",    TT[2]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Rx",    TT[3]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Ry",    TT[4]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Rz",    TT[5]) )

        /*   !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@OutputBipolar", m_Calibration->BiPolar) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@OutputRange", m_Calibration->VoltageRange) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@HWTempComp", m_Calibration->HWTempComp))*/

    {
        CMN_LOG_CLASS_INIT_ERROR << "failed to load config " << std::endl;
        return false;
    }	
    else {
        // set output units to calibration file defaults				

        for (int i = 0; i < 6 ; i++){
            m_Calibration->BasicTransform.TT[i]=TT[i];
        }

    }
    /*		else if(atts.element().key()==std::string("BiasSlope")){
    DAQFTCLIBRARY::Separate((char*)(atts.element().value().c_str()),cal->rt.bias_slopes,(unsigned short)(cal->rt.NumChannels-1));
    cal->TempCompAvailable = TRUE;
    }
    else if(atts.element().key()==std::string("GainSlope")){
    DAQFTCLIBRARY::Separate((char*)(atts.element().value().c_str()),cal->rt.gain_slopes,(unsigned short)(cal->rt.NumChannels-1));
    cal->TempCompAvailable = TRUE;
    }
    else if(atts.element().key()==std::string("Thermistor")){
    cal->rt.thermistor = (float) atof(atts.element().value().c_str());	
    }
    */

    //Now all the axes:

    char xmlContext[100];

    m_Calibration->rt.NumAxes=0;
    bool end = false;
    std::string aname;
    //find number of axes?
    while (!end) { 
        sprintf(xmlContext, "/FTSensor/Calibration/Axis[%d]", m_Calibration->rt.NumAxes+1);
        if (!config.GetXMLValue(xmlContext, "@Name", aname) ){
            end = true;
        }
        else { 
            m_Calibration->AxisNames[m_Calibration->rt.NumAxes]=DAQFTCLIBRARY::ATI_strdup(aname.c_str()); 
            m_Calibration->rt.NumAxes++;
        }
    }

    CMN_LOG_CLASS_INIT_DEBUG << "number of axis " << m_Calibration->rt.NumAxes<< std::endl;

    int axis = 0;
    for ( axis = 0; axis < 	m_Calibration->rt.NumAxes ; axis++) {

        m_Calibration->MaxLoads[axis]=0;  //default value just in case

        double scale = 1;
        sprintf(xmlContext, "/FTSensor/Calibration/Axis[%d]", axis+1);
        if (!config.GetXMLValue(xmlContext, "@scale", scale)  ){
            CMN_LOG_CLASS_INIT_ERROR << "failed to load config " << calFile<< std::endl;
            return false;    
        }

        std::string values;
        if(!config.GetXMLValue(xmlContext, "@values", values) ) {
            CMN_LOG_CLASS_INIT_ERROR << "failed to load config " << calFile<< std::endl;
            return false;    
        }
        double max=0;
        if(!config.GetXMLValue(xmlContext, "@max", max) ) {
            CMN_LOG_CLASS_INIT_ERROR << "failed to load config " << calFile<< std::endl;
            return false;    
        }
        else {
            m_Calibration->MaxLoads[axis] = max;
        }

        float temparray[8];

        DAQFTCLIBRARY::Separate((char *)(values.c_str()),temparray,(unsigned short)(m_Calibration->rt.NumChannels-1));
        for(int j = 0 ; j < m_Calibration->rt.NumChannels-1; j++) {
            m_Calibration->BasicMatrix[axis][j]=temparray[j]/scale;
        }
    }	
    CMN_LOG_CLASS_INIT_VERBOSE << "Configured : " << calFile<< std::endl;
    return true;
}

int devFTSensorATI::ReadSingleGaugePoint( float64 gaugeValues[] )
{

    unsigned int i; /*generic loop/array index*/
    if ( m_hiHardware->ReadSingleSample( gaugeValues ) )
    {
        m_sErrorInfo = m_hiHardware->GetErrorInfo();
        return -1;
    }

	
	//for (int ind = 0; ind < m_hiHardware->GetNumChannels(); ind++){
	//	std::cout << std::fixed << std::setprecision(3) << ind << " " << gaugeValues[ind] << "   ";
	//}
	//std::cout << std::endl;
	

    /*
    precondition: gaugeValues has the most recent gauge readings
    postcondition: will have returned 2 if any gauge is saturated. otherwise, i=number of gauges
    */
    bool gaugeSaturation = false;
    for( i = 0; i < m_hiHardware->GetNumChannels(); i++ )
    {
        if ( ( gaugeValues[i] >= m_dUpperSaturationVoltage ) || ( gaugeValues[i] <= m_dLowerSaturationVoltage ) )
        {
            m_sErrorInfo = std::string("Gauge Saturation");
            gaugeSaturation = true;    
        }		
    }
    if (gaugeSaturation)
        return 2;
    else 
        return 0;

	
}

std::string devFTSensorATI::GetErrorInfo()
{
    return m_sErrorInfo;
}

int devFTSensorATI::StartSingleSampleAcquisition(const std::string & deviceName, 
                                                 float64 sampleFrequency, 
                                                 int averaging, 
                                                 int firstChannel, 
                                                 bool useTempComp )
{
    int numChannels = NUM_STRAIN_GAUGES + ( (useTempComp)?1:0 );
    int status; /*status of starting the hardware acquisition*/
    status = m_hiHardware->ConfigSingleSampleTask( sampleFrequency, 
        averaging, 
        deviceName, 
        firstChannel, 
        numChannels, 
        m_iMinVoltage, 
        m_iMaxVoltage );
    if ( status )
    {
        if ( status < 0 ) /*hardware error*/{
            m_sErrorInfo = m_hiHardware->GetErrorInfo();
            std::cerr<<m_sErrorInfo<<std::endl;
        }else{ /*hardware warning*/
            m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
        }
        return -1;
    }
    /*set calibration's use of temp comp*/
    if ( NULL != m_Calibration )
        m_Calibration->cfg.TempCompEnabled = useTempComp;
    return 0;
}

std::string devFTSensorATI::GetSerialNumber(void) const
{
    if ( NULL == m_Calibration )
    {
        return std::string( "" );
    }
    return std::string( m_Calibration->Serial );
}

std::string devFTSensorATI::GetCalibrationDate(void) const
{
    if ( NULL == m_Calibration )
    {
        return std::string("");
    }
    return std::string( m_Calibration->CalDate );
}

float64 devFTSensorATI::GetMaxLoad( int axisIndex ) const
{
    if ( NULL == m_Calibration )
    {
        return 0;
    }
    float retVal; /*the return value*/
    /*find the maximum load in the current output units, not necessarily the same as the
    calibration units*/
    retVal = m_Calibration->MaxLoads[axisIndex]; /*this is the max load in the calibration
                                                 units*/
    if ( axisIndex < FIRST_TORQUE_INDEX ) /*this is a force axis, convert to output force units*/
    {
        retVal *= DAQFTCLIBRARY::ForceConv( m_Calibration->cfg.ForceUnits ) / DAQFTCLIBRARY::ForceConv( m_Calibration->ForceUnits );
    } else /*this is a torque axis, convert to output torque units*/
    {
        retVal *= DAQFTCLIBRARY::TorqueConv( m_Calibration->cfg.TorqueUnits ) / DAQFTCLIBRARY::TorqueConv( m_Calibration->TorqueUnits );
    }
    return retVal;
}

std::string devFTSensorATI::GetForceUnits(void ) const
{
    if ( NULL == m_Calibration )
        return std::string( "" );
    return std::string( m_Calibration->cfg.ForceUnits );
}


int devFTSensorATI::SetForceUnits(const std::string& forceUnits )
{
    char forceUnitsCString[WHOLE_LOTTA_CHARACTERS]; /*c-string representation of force-units*/
    ConvertStringToCString( forceUnits, forceUnitsCString, WHOLE_LOTTA_CHARACTERS );	
    return DAQFTCLIBRARY::SetForceUnits( m_Calibration, forceUnitsCString );
}

int devFTSensorATI::SetTorqueUnits( const std::string&  torqueUnits )
{
    char torqueUnitsCString[WHOLE_LOTTA_CHARACTERS]; /*c-string representation of torque-units*/
    ConvertStringToCString( torqueUnits, torqueUnitsCString, WHOLE_LOTTA_CHARACTERS );
    return DAQFTCLIBRARY::SetTorqueUnits( m_Calibration, torqueUnitsCString );
}

std::string devFTSensorATI::GetTorqueUnits(void ) const
{
    if ( NULL == m_Calibration )
        return std::string( "" );
    return std::string( m_Calibration->cfg.TorqueUnits );
}

bool devFTSensorATI::GetTempCompAvailable()
{
    if ( NULL == m_Calibration )
        return false;	
    return ( 0 != m_Calibration->TempCompAvailable );
}

//int devFTSensorATI::SetTempCompEnabled( bool EnableTempComp )
//{		
//	m_hiHardware->SetNumChannels( NUM_STRAIN_GAUGES + ( (EnableTempComp)?1:0 ) );
//	return SetTempComp( m_Calibration, (int) EnableTempComp );
//}

bool devFTSensorATI::GetTempCompEnabled()
{
    if ( NULL == m_Calibration )
        return false;
    return ( 0 != m_Calibration->cfg.TempCompEnabled );
}

int devFTSensorATI::ToolTransform( float64 transformVector[], const std::string & distanceUnits, const std::string& angleUnits )
{
    float tempTransforms[6]; /*unmanaged array of transformation values*/
    char cstrDistUnits[WHOLE_LOTTA_CHARACTERS]; /*c-string style distance units string*/
    char cstrAngleUnits[WHOLE_LOTTA_CHARACTERS]; /*c-string sytle angle units string*/
    int i; /*generic loop/array index*/
    ConvertStringToCString( distanceUnits, cstrDistUnits, WHOLE_LOTTA_CHARACTERS );
    ConvertStringToCString( angleUnits, cstrAngleUnits, WHOLE_LOTTA_CHARACTERS );
    /*
    precondition: transformVector has the transformation values
    postcondition: tempTransforms has a copy of transformVector, i = NUM_FT_AXES
    */
    for ( i = 0; i < NUM_FT_AXES; i++ )
    {
        tempTransforms[i] = (float)transformVector[i];
    }
    return DAQFTCLIBRARY::SetToolTransform( m_Calibration, tempTransforms, cstrDistUnits, cstrAngleUnits );
}

int devFTSensorATI::GetTransformVector( float64 transformVector[] )
{
    if ( NULL == m_Calibration )
        return 1; /*calibration not initialized*/
    int i; /*generic loop/array index*/
    for ( i = 0; i < NUM_FT_AXES; i++ )
    {
        transformVector[i] = m_Calibration->cfg.UserTransform.TT[i];
    }
    return 0;
}

std::string devFTSensorATI::GetTransformDistanceUnits()
{
    if ( NULL == m_Calibration ) 
        return std::string( "" );
    return std::string( m_Calibration->cfg.UserTransform.DistUnits );
}

std::string devFTSensorATI::GetTransformAngleUnits()
{
    if ( NULL == m_Calibration )
        return std::string ( "" );
    return std::string( m_Calibration->cfg.UserTransform.AngleUnits );
}

std::string devFTSensorATI::GetBodyStyle()
{
    if ( NULL == m_Calibration )
        return std::string( "" );
    return std::string( m_Calibration->BodyStyle );
}

std::string devFTSensorATI::GetCalibrationType()
{
    if ( NULL == m_Calibration )
        return std::string ( "" );
    return std::string ( m_Calibration->PartNumber );
}

//int devFTSensorATI::GetWorkingMatrix( 
//							float64 matrix [ati_ARRAY_DEC2(6,6)] )
//{
//	if ( NULL == m_Calibration )
//		return 1;
//	//float tempMatrix __nogc[ NUM_MATRIX_ELEMENTS ];
//	/*get matrix returns a one-dimensional array that you have
//													to parse into a matrix*/
//	int i, j; /*generic loop/aray indices*/
//	//GetMatrix( m_Calibration, tempMatrix );
//	for ( i = 0; i < NUM_FT_AXES; i++ )
//	{
//		for( j = 0; j < NUM_STRAIN_GAUGES; j++ )
//		{
//			matrix[ati_ARRAY_REF(i,j)] = m_Calibration->
//						rt.working_matrix[i][j];// tempMatrix[i * NUM_STRAIN_GAUGES + j];
//		}
//	}
//	return 0;
//}


//THIS IS A BLOCKING CALL
 //! \warning if the readings are saturated the conversion might produce odd results.
int devFTSensorATI::ReadSingleFTRecord( double readings [] )
{
    if ( NULL == m_Calibration )
        return 1;
    int retVal = 0; /*the return value*/
    float64 gcVoltages[ NUM_STRAIN_GAUGES + 1]; /*allow an extra reading for the thermistor*/	
    float nogcVoltages [NUM_STRAIN_GAUGES + 1]; /*non-gc version for passing to c library*/
    int i; /*generic loop/array index*/
    float tempResult [NUM_FT_AXES]; /*copy of readings read using C library*/
    retVal = ReadSingleGaugePoint( gcVoltages );

	//std::cerr<<std::setprecision(3)<<std::setw(8) << (float)gcVoltages[0]<<",  "
 //                               <<(float)gcVoltages[1]<<",  "
 //                               <<(float)gcVoltages[2]<<",  "
 //                               <<(float)gcVoltages[3]<<",  "
 //                               <<(float)gcVoltages[4]<<",  "
	//							<<(float)gcVoltages[5]<< "      \r";

//    if (retVal == 2) {
//        CMN_LOG_CLASS_INIT_ERROR<<"||Saturated"<<std::endl;
//    }
//    else
//        CMN_LOG_CLASS_INIT_ERROR<<std::endl;

    /*
    precondition: gcVoltages has the voltages from the DAQ card
    postcondition: nogcVoltages has a copy of the data in gcVoltages, i = NUM_STRAIN_GAUGES + 1
    */
    for ( i = 0; i <= NUM_STRAIN_GAUGES; i++ )
    {
        nogcVoltages[i] = (float)gcVoltages[i];
    }

    DAQFTCLIBRARY::ConvertToFT( m_Calibration, nogcVoltages, tempResult );
    /*
    precondition: tempResult has f/t values
    postcondition: readings has copy of f/t values, i = NUM_FT_AXES
    */
    for ( i = 0; i < NUM_FT_AXES; i++ )
    {
        readings[i] = tempResult[i];
    }

    if ( retVal ){ /*check for error when reading from hardware, -1 is real bad*/
        if ( 2 == retVal ) 
            return 2; /*saturation*/
        return 1;  /*other error*/
    }

    return retVal;

}

int devFTSensorATI::BiasCurrentLoadFromVoltages(const vctDoubleVec& voltages) {

    if ( NULL == m_Calibration )
        return 1; /*calibration not initialized*/
    float nogcVoltages[NUM_STRAIN_GAUGES]; /*voltages that can be passsed to unmanaged c library code*/
    int retVal = 0;

    /*
    precondition: curVoltages has current reading from hardware
    postcondition: nogcVoltages has a copy of the reading
    */
    for ( int i = 0; i < voltages.size(); i++ )
    {
        nogcVoltages[i] = (float)voltages[i];
    }
    DAQFTCLIBRARY::Bias( m_Calibration, nogcVoltages );
    return retVal;

}

int devFTSensorATI::BiasCurrentLoad()
{
    if ( NULL == m_Calibration )
        return 1; /*calibration not initialized*/
    float64 curVoltages[NUM_STRAIN_GAUGES + 1]; /*the current strain gauge load*/
    float nogcVoltages[NUM_STRAIN_GAUGES]; /*voltages that can be passsed to unmanaged c library code*/
    int retVal;
    int i; /*generic loop/array index*/
    retVal = ReadSingleGaugePoint( curVoltages );
    if ( retVal ) return retVal; /*error reading from hardware*/
    /*
    precondition: curVoltages has current reading from hardware
    postcondition: nogcVoltages has a copy of the reading
    */
    for ( i = 0; i < NUM_STRAIN_GAUGES; i++ )
    {
        nogcVoltages[i] = (float)curVoltages[i];
    }
    DAQFTCLIBRARY::Bias( m_Calibration, nogcVoltages );	
    return retVal;
}

int devFTSensorATI::BiasKnownLoad( float64 biasVoltages [] )
{
    if ( NULL == m_Calibration )
        return 1;
    float nogcVoltages [NUM_STRAIN_GAUGES]; /*version of bias voltages which can be passed to the c library*/
    int i; /*generic loop/array index*/
    /*
    precondition: biasVoltages has the known bias voltages
    postcondition: nogcVoltages is a copy of biasVoltages, i = NUM_STRAIN_GAUGES
    */
    for ( i = 0; i < NUM_STRAIN_GAUGES; i++ )
    {
        nogcVoltages[i] = (float)biasVoltages[i];
    }
    DAQFTCLIBRARY::Bias( m_Calibration, nogcVoltages );
    return 0;
}

int devFTSensorATI::StopAcquisition()
{
    if ( m_hiHardware->StopCollection() )
        return -1;
    return 0;
}

void devFTSensorATI::ConvertStringToCString(const std::string & sourceString, char destString[], unsigned int destSize )
{
    unsigned int i; /*generic loop/array index*/
    for ( i = 0; ( i < destSize ) && ( i < sourceString.size() ); i++ )
    {
        destString[i] = sourceString.c_str()[i];
    }
    destString[i] = '\0';
}

int devFTSensorATI::StartBufferedAcquisition( const std::string & deviceName, float64 sampleFrequency, int averaging,
                                             int firstChannel, bool useTempComp, int bufferSize )
{
    int numChannels = NUM_STRAIN_GAUGES + ( useTempComp?1:0 );
    int32 status = m_hiHardware->ConfigBufferTask( sampleFrequency, averaging, deviceName, firstChannel, numChannels,
        m_iMinVoltage, m_iMaxVoltage, bufferSize );		
    if ( status ) 
    {
        m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
        return -1;
    }
    /*set calibration's use of temp comp*/
    if ( NULL != m_Calibration )
        m_Calibration->cfg.TempCompEnabled = useTempComp;
    return 0;
}

int devFTSensorATI::ReadBufferedFTRecords( int numRecords, double readings [] )
{
    if ( NULL == m_Calibration ) /*invalid calibration*/
        return 1;	
    long status; /*the status of hardware reads*/
    int i, j; /*generic loop/array indices*/
    int numGauges = NUM_STRAIN_GAUGES + ( GetTempCompEnabled()?1:0 );
    unsigned int numGaugeValues = numRecords * numGauges;
    /*the number of individual gauge readings*/	
    float64 *gaugeValues = new float64 [numGaugeValues]; 	
    /*the gauge readings which are fed to the c library*/
    float currentGaugeReadings [ NUM_STRAIN_GAUGES + 1 ]; /*current gauge reading*/
    float currentFTValues [ NUM_FT_AXES ]; /*current f/t reading*/
    status = m_hiHardware->ReadBufferedSamples( numRecords, gaugeValues );		
    if ( status )
    {
        m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
        delete []gaugeValues; if ( NULL == m_Calibration )
            return 1;
        int retVal = 0; /*the return value*/
        float64 gcVoltages[ NUM_STRAIN_GAUGES + 1]; /*allow an extra reading for the thermistor*/
        float nogcVoltages [NUM_STRAIN_GAUGES + 1]; /*non-gc version for passing to c library*/
        int i; /*generic loop/array index*/
        float tempResult [NUM_FT_AXES]; /*copy of readings read using C library*/
        retVal = ReadSingleGaugePoint( gcVoltages );

    //      CMN_LOG_CLASS_INIT_ERROR<<(float)gcVoltages[0]<<","
    //                                <<(float)gcVoltages[1]<<","
    //                                <<(float)gcVoltages[2]<<","
    //                                <<(float)gcVoltages[3]<<","
    //                                <<(float)gcVoltages[4]<<","
    //                                <<(float)gcVoltages[5];



    //    if (retVal == 2) {
    //        CMN_LOG_CLASS_INIT_ERROR<<"||Saturated"<<std::endl;
    //    }
    //    else
    //        CMN_LOG_CLASS_INIT_ERROR<<std::endl;

        /*
        precondition: gcVoltages has the voltages from the DAQ card
        postcondition: nogcVoltages has a copy of the data in gcVoltages, i = NUM_STRAIN_GAUGES + 1
        */
        for ( i = 0; i <= NUM_STRAIN_GAUGES; i++ )
        {
            nogcVoltages[i] = (float)gcVoltages[i];
        }

        DAQFTCLIBRARY::ConvertToFT( m_Calibration, nogcVoltages, tempResult );
        /*
        precondition: tempResult has f/t values
        postcondition: readings has copy of f/t values, i = NUM_FT_AXES
        */
        for ( i = 0; i < NUM_FT_AXES; i++ )
        {
            readings[i] = tempResult[i];
        }

        if ( retVal ){ /*check for error when reading from hardware, -1 is real bad*/
            if ( 2 == retVal )
                return 2; /*saturation*/
            return 1;  /*other error*/
        }

        return retVal;
        return (int) status;		
    }
    /*
    precondition: gaugeValues has the buffered gauge readings, numGauges has the
    number of active gauges (6 or 7)
    postcondition: currentGaugeReadings contains the last gauge reading,
    currentFTValues contains the last ft value, readings contains all ft values,
    i = numRecords, j = NUM_FT_AXES, or the function will have already returned due
    to gauge saturation.
    */
    for ( i = 0; i < numRecords; i++ )
    {
        /*
        precondition: i is the number of the f/t record we are currently calculating.
        postcondition: currentGaugeReadings contains the i'th gauge readings,
        j = numGauges, or the function will have returned due to gauge saturation
        */
        for ( j = 0; j < numGauges; j++ )
        {
            currentGaugeReadings[j] = (float)gaugeValues[ j + ( i * numGauges ) ];
            /*check for saturation*/
            if ( m_dUpperSaturationVoltage < currentGaugeReadings[j] ){
                delete []gaugeValues;
                return 2;
            }
            if ( m_dLowerSaturationVoltage > currentGaugeReadings[j] ){
                delete []gaugeValues;			
                return 2;
            }
        }
        DAQFTCLIBRARY::ConvertToFT( m_Calibration, currentGaugeReadings, currentFTValues );
        /*
        precondition: currentFTValues has the i'th ft reading from the buffer
        postcondition: j = NUM_FT_AXES, the i'th f/t record in readings contains the values
        from currentFTValues
        */
        for ( j = 0; j < NUM_FT_AXES; j++ )
        {
            readings[ j + ( i * NUM_FT_AXES ) ] = currentFTValues[j];
        }
    }

    delete []gaugeValues;

    return 0;

}


int devFTSensorATI::GetBiasVector( double biasVector[] )
{
    if ( NULL == m_Calibration )
        return 1;
    int i; /*generic loop/array index*/
    /*
    precondition: m_Calibration is a valid calibration
    postcondition: i = NUM_STRAIN_GAUGES, biasVector contains a copy of the calibration's
    bias vector
    */
    for ( i = 0; i < NUM_STRAIN_GAUGES; i++ )
    {
        biasVector[i] = m_Calibration->rt.bias_vector[i];
    }
    return 0;
}

float64 devFTSensorATI::GetThermistorValue()
{
    if ( NULL == m_Calibration ) return 0;
    return m_Calibration->rt.thermistor;
}

float64 devFTSensorATI::GetBiasSlope( int index )
{
    if ( NULL == m_Calibration ) return 0;
    return m_Calibration->rt.bias_slopes[index];
}

float64 devFTSensorATI::GetGainSlope( int index )
{
    if ( NULL == m_Calibration ) return 0;
    return m_Calibration->rt.gain_slopes[index];
}

/*july.22.2005 - ss - added SetConnectionMode*/
void devFTSensorATI::SetConnectionMode( ATIDAQHardwareInterface::ConnectionType connType )
{
    m_hiHardware->SetConnectionMode( connType );
}

/*july.22.2005 - ss - added GetConnectionmode*/
ATIDAQHardwareInterface::ConnectionType devFTSensorATI::GetConnectionMode( )
{
    return (ATIDAQHardwareInterface::ConnectionType)m_hiHardware->GetConnectionMode();
}

/*aug.5.2005a - ss - added GetHardwareTempComp*/
bool devFTSensorATI::GetHardwareTempComp( )
{
    if ( m_Calibration->HWTempComp ) return true;
    return false;
}



void devFTSensorATI::ToStream(std::ostream & outputStream) const
{
    if (m_Calibration==NULL) return;

    outputStream << "\nBTangle units : " <<m_Calibration->BasicTransform.AngleUnits
        <<"\nBT distance units: " <<m_Calibration->BasicTransform.DistUnits
        <<"\nUTangle    units: " <<m_Calibration->cfg.UserTransform.AngleUnits			
        <<"\nUTdistance units: " <<m_Calibration->cfg.UserTransform.DistUnits	
        <<"\nForce		units: " <<m_Calibration->ForceUnits 
        <<"\nTorque		units: " <<m_Calibration->TorqueUnits
        <<"\nNumOfChannels: " <<m_Calibration->rt.NumChannels 
        <<"\nBodyStyle		: " <<m_Calibration->BodyStyle 			
        <<"\nFamily			: " <<m_Calibration->Family 			
        <<"\nSerial			: " <<m_Calibration->Serial 			
        <<"\nPartNumber		: " <<m_Calibration->PartNumber 			
        <<"\nCalDate		: " <<m_Calibration->CalDate 	
        <<"\nHWTempComp		: " <<m_Calibration->HWTempComp 
        <<"\nVoltageRange	: " <<m_Calibration->VoltageRange
        <<"\nIsBiPolar		: "<<m_Calibration->BiPolar

        <<"\n";

    int i;
    for(i=0;i<MAX_AXES;i++) {
        outputStream <<"\nAxis:"<<(m_Calibration->AxisNames[i])
            <<", MaxLoad: "<<m_Calibration->MaxLoads[i]
        <<", Basic Matrix :\n";
        for(int j=0;j<m_Calibration->rt.NumChannels-1;j++) {
            outputStream <<	m_Calibration->BasicMatrix[i][j]<<", ";
        }

    }
    outputStream<<"\nBasicTransform :    ";
    for(i=0;i<6;i++) {
        outputStream <<m_Calibration->BasicTransform.TT[i]<<" ,";
    }

    //TODO : bias slope...., gain slope, thermistor

    outputStream<<"\n";

}

int devFTSensorATI::GetForcefromVoltages(const vctDoubleVec &voltage, vctDoubleVec &ft ) {

    if ( NULL == m_Calibration )
        return 1;
    int retVal = 0; /*the return value*/
    float nogcVoltages [NUM_STRAIN_GAUGES + 1]; /*non-gc version for passing to c library*/
    int i; /*generic loop/array index*/
    float tempResult [NUM_FT_AXES]; /*copy of readings read using C library*/

     /*
    precondition: gaugeValues has the most recent gauge readings
    postcondition: will have returned 2 if any gauge is saturated. otherwise, i=number of gauges
    */
    bool gaugeSaturation = false;
    for( i = 0; i < m_hiHardware->GetNumChannels(); i++ )
    {
        if ( ( voltage[i] >= m_dUpperSaturationVoltage ) || ( voltage[i] <= m_dLowerSaturationVoltage ) )
        {
            m_sErrorInfo = std::string("Gauge Saturation");
            gaugeSaturation = true;
        }
    }
    if (gaugeSaturation)
        retVal = 2;

//      CMN_LOG_CLASS_INIT_ERROR<<(float)gcVoltages[0]<<","
//                                <<(float)gcVoltages[1]<<","
//                                <<(float)gcVoltages[2]<<","
//                                <<(float)gcVoltages[3]<<","
//                                <<(float)gcVoltages[4]<<","
//                                <<(float)gcVoltages[5];



//    if (retVal == 2) {
//        CMN_LOG_CLASS_INIT_ERROR<<"||Saturated"<<std::endl;
//    }
//    else
//        CMN_LOG_CLASS_INIT_ERROR<<std::endl;

    /*
    precondition: gcVoltages has the voltages from the DAQ card
    postcondition: nogcVoltages has a copy of the data in gcVoltages, i = NUM_STRAIN_GAUGES + 1
    */
    for ( i = 0; i <= NUM_STRAIN_GAUGES; i++ )
    {
        nogcVoltages[i] = (float)voltage[i];
    }

    DAQFTCLIBRARY::ConvertToFT( m_Calibration, nogcVoltages, tempResult );
    /*
    precondition: tempResult has f/t values
    postcondition: readings has copy of f/t values, i = NUM_FT_AXES
    */
    for ( i = 0; i < NUM_FT_AXES; i++ )
    {
        ft[i] = tempResult[i];
    }

    return retVal;

}
