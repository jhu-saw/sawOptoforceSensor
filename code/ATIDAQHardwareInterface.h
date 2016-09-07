/*ATIDAQHardwareInterface.h
header file for ATIDAQHardwareInterface Class

history
Dec.13.2004 - Sam Skuce (ATI Industrial Automation) - Initial Revision Started.  Works
        only with NIDAQmx hardware.
July.22.2005 - ss - added support for user setting connection mode

September 2008 removed .NET related stuff - (marcin)

*/
#ifndef _ATIDAQHARDWAREINTERFACE_H
#define _ATIDAQHARDWAREINTERFACE_H
#include <string>
//#include <stdlib.h>

#ifdef _WIN32 // _WIN32 is defined by all Windows 32 compilers, but not by others.

#include <C:\Program Files (x86)\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include\NIDAQmx.h>

class ATIDAQHardwareInterface
{
public:
        ATIDAQHardwareInterface(); /*constructor*/
        ~ATIDAQHardwareInterface(); /*destructor*/

        enum ConnectionType{
                DIFFERENTIAL = DAQmx_Val_Diff,
                REFERENCED_SINGLE_ENDED = DAQmx_Val_RSE,
                NON_REFERENCED_SINGLE_ENDED = DAQmx_Val_NRSE,
                PSEUDO_DIFFERENTIAL = DAQmx_Val_PseudoDiff,
        };

        /*
        int32 ConfigSingleSampleTask( float64 sampleRate, int averaging, String * deviceName, int firstChannel,
                int numChannels, float64 minVoltage, float64 maxVoltage )
        configures the single sample task
        arguments:
                sampleRate - the rate at which to sample the hardware
                averaging - the number of raw samples to average together to make one output sample.  useful for
                        reducing noise
                deviceName - the name of the NI-DAQmx device which the transducer is attached to
                firstChannel - the lowest-numbered channel that the transducer is attached to
                numChannels - the number of channels that the transducer uses.  should be 6 if temperature compensation
                        is not being used, 7 if it is.
                minVoltage - the minimum voltage output by the transducer, usually -10 or -5
                maxVoltage - the maximum voltage output by the transducer, usually 10 or 5
        returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
        */
        int32 ConfigSingleSampleTask( float64 sampleRate, int averaging, const std::string& deviceName, int firstChannel,
                int numChannels, int minVoltage, int maxVoltage );

        /*
        int32 StopCollection();
        stops and clears the currently running DAQ task, freeing it's resources
        returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
        */
        int32 StopCollection();

        /*
        int32 ReadSingleSample( buffer )
        reads a single sample from the card, performing any averaging necessary.
        arguments:
                buffer - out - a single sample, containing 6 or 7 gauge values (7 if software temp comp is enabled)
        returns:
                NI-DAQmx return code ( 0 if success, <0 if error, >0 if warning )
        */
        int32 ReadSingleSample( float64  buffer[] );

        /*
        String GetErrorInfo()
        get a description of the most recent error
        returns:
                NIDAQmx Error Info
        side effects:
                sets m_sErrorInfo to description
        */
        std::string GetErrorInfo();

        /*
        GetMinVoltage()
        get the minimum voltage of the transducer
        returns:
                the minimum voltage of the transducer
        */
        int GetMinVoltage() { return m_iMinVoltage; }

        /*
        GetMaxVoltage()
        get the maximum voltage of the transducer
        returns:
                the maximum voltage of the transducer
        */
        int GetMaxVoltage() { return m_iMaxVoltage; }

        /*
        GetFirstChannel()
        get the first channel that the transducer is connected to
        returns:
                the number of the first (lowest-numbered) channel that the transducer is connected
                to
        */
        unsigned int GetFirstChannel() { return m_uiFirstChannel; }

        /*
        GetDeviceName()
        get the name of the DAQ device that the transducer is connected to
        returns:
                the name of the device that the transducer is connected to
        */
        std::string GetDeviceName();

        /*
        GetNumChannels()
        get the number of channels used by the transducer, should be 6 if software temperature compensation
                is not used, or 7 if it is used.
        returns:
                the number of channels used by the transducer
        */
        unsigned int GetNumChannels() { return m_uiNumChannels; }

        /*
        GetSampleFrequency()
        get the frequency at which to sample the transducer
        returns:
                the frequency at which the transducer is sampled
        */
        float64 GetSampleFrequency() { return m_f64SamplingFrequency; }

        /*
        GetAveragingSamples()
        get the number of samples which are averaged together for noise reduction
        returns:
                the number of raw samples which are averaged together to form one output
                sample
        */
        unsigned int GetAveragingSamples() { return m_uiAveragingSize; }

        /*
        String * GetErrorCodeDescription( long errCode )
        get the description of a known error code
        arguments:
                errCode - the error code you wish to get a description for
        returns:
                a string with the description of the error
        */
        std::string GetErrorCodeDescription( long errCode );


        /*
        int32 ConfigBufferTask( float64 sampleRate, int averaging, String * deviceName, int firstChannel,
                int numChannels, float64 minVoltage, float64 maxVoltage, int bufferSize )
        configures and starts buffered acquisition task
        arguments:
                sampleRate - the rate at which to sample the hardware
                averaging - the number of raw samples to average together to make one output sample.  useful for
                        reducing noise
                deviceName - the name of the NI-DAQmx device which the transducer is attached to
                firstChannel - the lowest-numbered channel that the transducer is attached to
                numChannels - the number of channels that the transducer uses.  should be 6 if temperature compensation
                        is not being used, 7 if it is.
                minVoltage - the minimum voltage output by the transducer, usually -10 or -5
                maxVoltage - the maximum voltage output by the transducer, usually 10 or 5
                bufferSize - the size of the buffer to allocate for the acquisition.  This is the number of output samples
                        (after averaging) you want in the buffer, so the actual number of raw samples in the buffer will be
                        ( bufferSize * averaging )
        returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
        */
        int32 ConfigBufferTask( float64 sampleRate, int averaging, const std::string& deviceName, int firstChannel,
                int numChannels, int minVoltage, int maxVoltage, int bufferSize );
        /*
        int 32 ReadBufferedSamples( int numSamples, float64 buffer __gc[] )
        reads buffered samples from the DAQ hardware, performing any averaging necessary
        arguments:
                numSamples - the number of samples to output
                buffer - out - the averaged output samples from the DAQ hardware.
        */
        int32 ReadBufferedSamples( int numSamples, float64 buffer[] );

        /*
        july.22.2005 - ss  added
        void SetConnectionMode( int DAQConnMode )
        sets the DAQ connection mode
        arguments:
                DAQConnMode - the connection mode for the DAQ device to use
        side effects:
                sets m_iConnectionMode to DAQConnMode
        */
        void SetConnectionMode( int DAQConnMode );

        /*
        july.22.2005 - ss - added
        int GetConnectionMode( )
        get the DAQ connection mode
        returns:
                the connection mode of the DAQ device
        */
        int GetConnectionMode( );


private:
        TaskHandle*			m_thDAQTask;		/*the task which is used to get a data*/
        unsigned int		m_uiAveragingSize;	/*the number of samples to average together to smooth data*/
        float64				m_f64SamplingFrequency; /*the frequency at which to sample data, applies to both buffered and
                                                                                                        single-point acquisitions*/
        unsigned long		m_ulBufferedSize;	/*the buffer size to use with buffered continuous acquisition*/
        unsigned int		m_uiNumChannels;	/*the number of channels to sample, should always be either 6 (no software
                                                                                                temp comp.) or 7 (with software temp comp)*/
        std::string			m_sDeviceName ;		/*the name of the NI-DAQmx device which the transducer is attached to*/
        unsigned int		m_uiFirstChannel;	/*the first channel occupied by the transducer*/
        int					m_iMinVoltage;		/*the minimum voltage*/
        int					m_iMaxVoltage;		/*the maximum voltage*/
        std::string			m_sErrorInfo;		/*information about the last error*/
        int					m_iConnectionMode;	/*connection mode of the DAQ device - july.22.2005 - ss*/

        /*
        ati_strncpy( dest, source, len )
        copies a c-style string from one location to another, with argument-based bounds-checking.
        arguments:
                dest - out - contains string copied from source, or first (len - 1) characters, if source is
                        longer than (len - 1)
                source - the string to copy from
                len - the maximum number of characters that can safely be placed into dest (relying on caller to be honest here)
        returns: pointer to dest
        */
        char* ati_strncpy( char * dest, char const * source, unsigned int len );

        /*
        ati_strncat( dest, catsource, len )
        appends to a c-style string, with argument-based bounds checking.
        arguments:
                dest - in - a c-style string, with null terminator
                         - out - catsource has been appended to end, or is a string of length (len - 1), if
                                resulting string (dest + catsource)  is longer than (len - 1) characters
                catsource - the string to append to dest
                len - the maximum number of characters that dest can safely hold ( relying on caller to be honest here)
        returns:
                pointer to dest
        */
        char * ati_strncat( char * dest, char const * catsource, unsigned int len );



        /*
        ati_nstrlen( buffer, buffersize )
        find the length of a c-style string, with argument-based bounds checking
        arguments:
                buffer - the c-style string to find the length of
                buffersize - the total size of buffer, including room for the null-character at the end,
                        i.e. pass 1000, not 999, if buffer is a char[1000]
        returns:
                length of buffer, or buffersize if no null character found
        */
        unsigned long ati_nstrlen( char const * buffer, unsigned long buffersize );

        /*
         * Format routine for National Instruments channel names
         *
         * no meaningful return value
         */
        int32 ati_Channel_Name_Format( char * channelString, const std::string & m_sDeviceName );


};
//# ----------------------Linux--------------------------------------
#else

#include <comedi.h>
#include <comedilib.h>
typedef double float64;
typedef int    int32;
#define   DAQmx_Val_Diff     1
#define     DAQmx_Val_RSE       2
#define     DAQmx_Val_NRSE      3
#define     DAQmx_Val_PseudoDiff 4

class ATIDAQHardwareInterface
{
public:
        ATIDAQHardwareInterface(); /*constructor*/
        ~ATIDAQHardwareInterface(); /*destructor*/

        enum ConnectionType{
                DIFFERENTIAL = DAQmx_Val_Diff,
                REFERENCED_SINGLE_ENDED = DAQmx_Val_RSE,
                NON_REFERENCED_SINGLE_ENDED = DAQmx_Val_NRSE,
                PSEUDO_DIFFERENTIAL = DAQmx_Val_PseudoDiff,
        };

        /*
        int32 ConfigSingleSampleTask( float64 sampleRate, int averaging, String * deviceName, int firstChannel,
                int numChannels, float64 minVoltage, float64 maxVoltage )
        configures the single sample task
        arguments:
                sampleRate - the rate at which to sample the hardware
                averaging - the number of raw samples to average together to make one output sample.  useful for
                        reducing noise
                deviceName - the name of the NI-DAQmx device which the transducer is attached to
                firstChannel - the lowest-numbered channel that the transducer is attached to
                numChannels - the number of channels that the transducer uses.  should be 6 if temperature compensation
                        is not being used, 7 if it is.
                minVoltage - the minimum voltage output by the transducer, usually -10 or -5
                maxVoltage - the maximum voltage output by the transducer, usually 10 or 5
        returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
        */
        int32 ConfigSingleSampleTask( float64 sampleRate, int averaging, const std::string& deviceName, int firstChannel,
                int numChannels, int minVoltage, int maxVoltage );

        /*
        int32 StopCollection();
        stops and clears the currently running DAQ task, freeing it's resources
        returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
        */
        int32 StopCollection();

        /*
        int32 ReadSingleSample( buffer )
        reads a single sample from the card, performing any averaging necessary.
        arguments:
                buffer - out - a single sample, containing 6 or 7 gauge values (7 if software temp comp is enabled)
        returns:
                NI-DAQmx return code ( 0 if success, <0 if error, >0 if warning )
        */
        int32 ReadSingleSample( float64  buffer[] );

        /*
        String GetErrorInfo()
        get a description of the most recent error
        returns:
                NIDAQmx Error Info
        side effects:
                sets m_sErrorInfo to description
        */
        std::string GetErrorInfo();

        /*
        GetMinVoltage()
        get the minimum voltage of the transducer
        returns:
                the minimum voltage of the transducer
        */
        int GetMinVoltage() { return m_iMinVoltage; }

        /*
        GetMaxVoltage()
        get the maximum voltage of the transducer
        returns:
                the maximum voltage of the transducer
        */
        int GetMaxVoltage() { return m_iMaxVoltage; }

        /*
        GetFirstChannel()
        get the first channel that the transducer is connected to
        returns:
                the number of the first (lowest-numbered) channel that the transducer is connected
                to
        */
        unsigned int GetFirstChannel() { return m_uiFirstChannel; }

        /*
        GetDeviceName()
        get the name of the DAQ device that the transducer is connected to
        returns:
                the name of the device that the transducer is connected to
        */
        std::string GetDeviceName();

        /*
        GetNumChannels()
        get the number of channels used by the transducer, should be 6 if software temperature compensation
                is not used, or 7 if it is used.
        returns:
                the number of channels used by the transducer
        */
        unsigned int GetNumChannels() { return m_uiNumChannels; }

        /*
        GetSampleFrequency()
        get the frequency at which to sample the transducer
        returns:
                the frequency at which the transducer is sampled
        */
        float64 GetSampleFrequency() { return m_f64SamplingFrequency; }

        /*
        GetAveragingSamples()
        get the number of samples which are averaged together for noise reduction
        returns:
                the number of raw samples which are averaged together to form one output
                sample
        */
        unsigned int GetAveragingSamples() { return m_uiAveragingSize; }

        /*
        String * GetErrorCodeDescription( long errCode )
        get the description of a known error code
        arguments:
                errCode - the error code you wish to get a description for
        returns:
                a string with the description of the error
        */
        std::string GetErrorCodeDescription( long errCode );


        /*
        int32 ConfigBufferTask( float64 sampleRate, int averaging, String * deviceName, int firstChannel,
                int numChannels, float64 minVoltage, float64 maxVoltage, int bufferSize )
        configures and starts buffered acquisition task
        arguments:
                sampleRate - the rate at which to sample the hardware
                averaging - the number of raw samples to average together to make one output sample.  useful for
                        reducing noise
                deviceName - the name of the NI-DAQmx device which the transducer is attached to
                firstChannel - the lowest-numbered channel that the transducer is attached to
                numChannels - the number of channels that the transducer uses.  should be 6 if temperature compensation
                        is not being used, 7 if it is.
                minVoltage - the minimum voltage output by the transducer, usually -10 or -5
                maxVoltage - the maximum voltage output by the transducer, usually 10 or 5
                bufferSize - the size of the buffer to allocate for the acquisition.  This is the number of output samples
                        (after averaging) you want in the buffer, so the actual number of raw samples in the buffer will be
                        ( bufferSize * averaging )
        returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
        */
        int32 ConfigBufferTask( float64 sampleRate, int averaging, const std::string& deviceName, int firstChannel,
                int numChannels, int minVoltage, int maxVoltage, int bufferSize );
        /*
        int 32 ReadBufferedSamples( int numSamples, float64 buffer __gc[] )
        reads buffered samples from the DAQ hardware, performing any averaging necessary
        arguments:
                numSamples - the number of samples to output
                buffer - out - the averaged output samples from the DAQ hardware.
        */
        int32 ReadBufferedSamples( int numSamples, float64 buffer[] );

        /*
        july.22.2005 - ss  added
        void SetConnectionMode( int DAQConnMode )
        sets the DAQ connection mode
        arguments:
                DAQConnMode - the connection mode for the DAQ device to use
        side effects:
                sets m_iConnectionMode to DAQConnMode
        */
        void SetConnectionMode( int DAQConnMode );

        /*
        july.22.2005 - ss - added
        int GetConnectionMode( )
        get the DAQ connection mode
        returns:
                the connection mode of the DAQ device
        */
        int GetConnectionMode( );


private:
       // TaskHandle*			m_thDAQTask;		/*the task which is used to get a data*/
        comedi_t                * comediDev;
        comedi_range            range;
        lsampl_t                maxdata;
        lsampl_t                data;
        //trying to create a single instruction read:

          comedi_insn  		*insn;
          comedi_insnlist 	il;
          lsampl_t              *dataArray;



        unsigned int		m_uiAveragingSize;	/*the number of samples to average together to smooth data*/
        float64			m_f64SamplingFrequency; /*the frequency at which to sample data, applies to both buffered and
                                                                                                        single-point acquisitions*/
        unsigned long		m_ulBufferedSize;	/*the buffer size to use with buffered continuous acquisition*/
        unsigned int		m_uiNumChannels;	/*the number of channels to sample, should always be either 6 (no software
                                                                                                temp comp.) or 7 (with software temp comp)*/
        std::string		m_sDeviceName ;		/*the name of the NI-DAQmx device which the transducer is attached to*/
        unsigned int		m_uiFirstChannel;	/*the first channel occupied by the transducer*/
        int			m_iMinVoltage;		/*the minimum voltage*/
        int			m_iMaxVoltage;		/*the maximum voltage*/
        std::string		m_sErrorInfo;		/*information about the last error*/
        int			m_iConnectionMode;	/*connection mode of the DAQ device - july.22.2005 - ss*/

        /*
        ati_strncpy( dest, source, len )
        copies a c-style string from one location to another, with argument-based bounds-checking.
        arguments:
                dest - out - contains string copied from source, or first (len - 1) characters, if source is
                        longer than (len - 1)
                source - the string to copy from
                len - the maximum number of characters that can safely be placed into dest (relying on caller to be honest here)
        returns: pointer to dest
        */
        char* ati_strncpy( char * dest, char const * source, unsigned int len );

        /*
        ati_strncat( dest, catsource, len )
        appends to a c-style string, with argument-based bounds checking.
        arguments:
                dest - in - a c-style string, with null terminator
                         - out - catsource has been appended to end, or is a string of length (len - 1), if
                                resulting string (dest + catsource)  is longer than (len - 1) characters
                catsource - the string to append to dest
                len - the maximum number of characters that dest can safely hold ( relying on caller to be honest here)
        returns:
                pointer to dest
        */
        char * ati_strncat( char * dest, char const * catsource, unsigned int len );



        /*
        ati_nstrlen( buffer, buffersize )
        find the length of a c-style string, with argument-based bounds checking
        arguments:
                buffer - the c-style string to find the length of
                buffersize - the total size of buffer, including room for the null-character at the end,
                        i.e. pass 1000, not 999, if buffer is a char[1000]
        returns:
                length of buffer, or buffersize if no null character found
        */
        unsigned long ati_nstrlen( char const * buffer, unsigned long buffersize );

        /*
         * Format routine for National Instruments channel names
         *
         * no meaningful return value
         */
        int32 ati_Channel_Name_Format( char * channelString, const std::string & m_sDeviceName );


};
#endif

#endif
