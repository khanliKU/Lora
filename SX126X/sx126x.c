/**
 ******************************************************************************
 * @file		: sx126x.c
 * @author		: Kutlay Hanlý
 * @email		: khanli@ku.edu.tr
 * @date		: 21 Mar 2020
 * @brief		: TODO fill in
 ******************************************************************************
 ** radio notice applies to any and all portions of radio file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of radio file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      radio list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      radio list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Delta V nor the names of its contributors
 *      may be used to endorse or promote products derived from radio software
 *      without specific prior written permission.
 *
 * radio SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF radio SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sx126x.h"
/* Includes END --------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private define END --------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/*!
 * \brief Helper macro to create Interrupt objects only if the pin name is
 *        different from NC
 */
#define CreateDioPin( pinName, dio )                 \
            if( pinName == NC )                      \
            {                                        \
                dio = NULL;                          \
            }                                        \
            else                                     \
            {                                        \
                dio = new InterruptIn( pinName );    \
            }

/*!
 * \brief Helper macro to avoid duplicating code for setting dio pins parameters
 */
#define DioAssignCallback( dio, pinMode, callback )                    \
            if( dio != NULL )                                          \
            {                                                          \
                dio->mode( pinMode );                                  \
                dio->rise( radio, static_cast <Trigger>( callback ) );  \
            }

// radio code handles cases where assert_param is undefined
#ifndef assert_param
#define assert_param( ... )
#endif
/* Private macro END ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/*!
 * \brief Radio registers definition
 */
typedef struct
{
	uint16_t Addr ;                             //!< The address of the register
	uint8_t Value ;                            //!< The value of the register
} RadioRegisters_t ;

/* Private typedef END -------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/*!
 * \brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t FrequencyError = 0 ;

/*!
 * \brief Hold the status of the Image calibration
 */
static uint8_t ImageCalibrated = FALSE ;

/* Private variables END -----------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function prototypes END -------------------------------------------*/

/* Implementations -----------------------------------------------------------*/

void WaitOnBusy ( sx126x_t* radio )
{
	while ( radio->IO_callbacks->getRadioBUSY() )
	{
	}
}

/*!
 * \brief Used to block execution to give enough time to Busy to go up
 *        in order to respect Tsw, see datasheet 8.3.1
 */
void WaitOnCounter ()
{
	for ( uint8_t counter = 0 ; counter < 15 ; counter++ )
	{
//		__NOP () ;
	}
}

void Init ( sx126x_t* radio )
{
	CalibrationParams_t calibParam ;

	/*!
	 * \brief pin OPT is used to detect if the board has a TCXO or a XTAL
	 *
	 * OPT = 0 >> TCXO; OPT = 1 >> XTAL
	 */
//	DigitalIn OPT ( A3 ) ;
	Reset ( radio ) ;

//	IoIrqInit ( dioIrq ) ;

	Wakeup ( radio ) ;
	SetStandby ( radio, STDBY_RC ) ;

	if ( /*todo OPT ==*/0 )
	{
		SetDio3AsTcxoCtrl ( radio, TCXO_CTRL_1_7V, 320 ) ; //5 ms
		calibParam.Value = 0x7F ;
		Calibrate ( radio, calibParam ) ;
	}

	SetPollingMode ( radio ) ;

	AntSwOn ( radio ) ;
	SetDio2AsRfSwitchCtrl ( radio, TRUE ) ;

	radio->OperatingMode = MODE_STDBY_RC ;

	SetPacketType ( radio, PACKET_TYPE_LORA ) ;

#ifdef USE_CONFIG_PUBLIC_NETOWRK
	// Change LoRa modem Sync Word for Public Networks
	WriteReg( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF ) ;
	WriteReg( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF ) ;
#else
	// Change LoRa modem SyncWord for Private Networks
	WriteReg ( radio, REG_LR_SYNCWORD,
			( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF ) ;
	WriteReg ( radio, REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF ) ;
#endif
}

RadioOperatingModes_t GetOperatingMode ( sx126x_t* radio )
{
	return radio->OperatingMode ;
}

void CheckDeviceReady ( sx126x_t* radio )
{
	if ( ( GetOperatingMode ( radio ) == MODE_SLEEP )
			|| ( GetOperatingMode ( radio ) == MODE_RX_DC ) )
	{
		Wakeup ( radio ) ;
		// Switch is turned off when device is in sleep mode and turned on is all other modes
		AntSwOn ( radio ) ;
	}
}

void SetPayload ( sx126x_t* radio, uint8_t *payload, uint8_t size )
{
	WriteBuffer ( radio, 0x00, payload, size ) ;
}

uint8_t GetPayload ( sx126x_t* radio, uint8_t *buffer, uint8_t *size,
		uint8_t maxSize )
{
	uint8_t offset = 0 ;

	GetRxBufferStatus ( radio, size, &offset ) ;
	if ( *size > maxSize )
	{
		return 1 ;
	}
	ReadBuffer ( radio, offset, buffer, *size ) ;
	return 0 ;
}

void SendPayload ( sx126x_t* radio, uint8_t *payload, uint8_t size,
		uint32_t timeout )
{
	SetPayload ( radio, payload, size ) ;
	SetTx ( radio, timeout ) ;
}

uint8_t SetSyncWord ( sx126x_t* radio, uint8_t *syncWord )
{
	WriteRegister ( radio, REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 ) ;
	return 0 ;
}

void SetCrcSeed ( sx126x_t* radio, uint16_t seed )
{
	uint8_t buf [2] ;

	buf [0] = (uint8_t) ( ( seed >> 8 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( seed & 0xFF ) ;

	switch ( GetPacketType ( radio ) )
	{
	case PACKET_TYPE_GFSK :
		WriteRegister ( radio, REG_LR_CRCSEEDBASEADDR, buf, 2 ) ;
		break ;

	default :
		break ;
	}
}

void SetCrcPolynomial ( sx126x_t* radio, uint16_t polynomial )
{
	uint8_t buf [2] ;

	buf [0] = (uint8_t) ( ( polynomial >> 8 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( polynomial & 0xFF ) ;

	switch ( GetPacketType ( radio ) )
	{
	case PACKET_TYPE_GFSK :
		WriteRegister ( radio, REG_LR_CRCPOLYBASEADDR, buf, 2 ) ;
		break ;

	default :
		break ;
	}
}

void SetWhiteningSeed ( sx126x_t* radio, uint16_t seed )
{
	uint8_t regValue = 0 ;

	switch ( GetPacketType ( radio ) )
	{
	case PACKET_TYPE_GFSK :
		regValue = ReadReg ( radio, REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE ;
		regValue = ( ( seed >> 8 ) & 0x01 ) | regValue ;
		WriteReg ( radio, REG_LR_WHITSEEDBASEADDR_MSB, regValue ) ; // only 1 bit.
		WriteReg ( radio, REG_LR_WHITSEEDBASEADDR_LSB, (uint8_t) seed ) ;
		break ;

	default :
		break ;
	}
}

uint32_t GetRandom ( sx126x_t* radio )
{
	uint8_t buf [] =
	{ 0, 0, 0, 0 } ;

	// Set radio in continuous reception
	SetRx ( radio, 0 ) ;

	radio->wait_ms ( 1 ) ;

	ReadRegister ( radio, RANDOM_NUMBER_GENERATORBASEADDR, buf, 4 ) ;

	SetStandby ( radio, STDBY_RC ) ;

	return ( buf [0] << 24 ) | ( buf [1] << 16 ) | ( buf [2] << 8 ) | buf [3] ;
}

void SetSleep ( sx126x_t* radio, SleepParams_t sleepConfig )
{
#ifdef ADV_DEBUG
	printf("SetSleep ") ;
#endif

	AntSwOff ( radio ) ;

	WriteCommand ( radio, RADIO_SET_SLEEP, &sleepConfig.Value, 1 ) ;
	radio->OperatingMode = MODE_SLEEP ;
}

void SetStandby ( sx126x_t* radio, RadioStandbyModes_t standbyConfig )
{
#ifdef ADV_DEBUG
	printf("SetStandby ") ;
#endif
	WriteCommand ( radio, RADIO_SET_STANDBY, (uint8_t*) &standbyConfig, 1 ) ;
	if ( standbyConfig == STDBY_RC )
	{
		radio->OperatingMode = MODE_STDBY_RC ;
	}
	else
	{
		radio->OperatingMode = MODE_STDBY_XOSC ;
	}
}

void SetFs ( sx126x_t* radio )
{
#ifdef ADV_DEBUG
	printf("SetFs ") ;
#endif
	WriteCommand ( radio, RADIO_SET_FS, 0, 0 ) ;
	radio->OperatingMode = MODE_FS ;
}

void SetTx ( sx126x_t* radio, uint32_t timeout )
{
	uint8_t buf [3] ;

	radio->OperatingMode = MODE_TX ;

#ifdef ADV_DEBUG
	printf("SetTx ") ;
#endif

	buf [0] = (uint8_t) ( ( timeout >> 16 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( ( timeout >> 8 ) & 0xFF ) ;
	buf [2] = (uint8_t) ( timeout & 0xFF ) ;
	WriteCommand ( radio, RADIO_SET_TX, buf, 3 ) ;
}

void SetRxBoosted ( sx126x_t* radio, uint32_t timeout )
{
	uint8_t buf [3] ;

	radio->OperatingMode = MODE_RX ;

#ifdef ADV_DEBUG
	printf("SetRxBoosted ") ;
#endif

	WriteReg ( radio, REG_RX_GAIN, 0x96 ) ; // max LNA gain, increase current by ~2mA for around ~3dB in sensivity

	buf [0] = (uint8_t) ( ( timeout >> 16 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( ( timeout >> 8 ) & 0xFF ) ;
	buf [2] = (uint8_t) ( timeout & 0xFF ) ;
	WriteCommand ( radio, RADIO_SET_RX, buf, 3 ) ;
}

void SetRx ( sx126x_t* radio, uint32_t timeout )
{
	uint8_t buf [3] ;

	radio->OperatingMode = MODE_RX ;

#ifdef ADV_DEBUG
	printf("SetRx ") ;
#endif

	buf [0] = (uint8_t) ( ( timeout >> 16 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( ( timeout >> 8 ) & 0xFF ) ;
	buf [2] = (uint8_t) ( timeout & 0xFF ) ;
	WriteCommand ( radio, RADIO_SET_RX, buf, 3 ) ;
}

void SetRxDutyCycle ( sx126x_t* radio, uint32_t rxTime, uint32_t sleepTime )
{
	uint8_t buf [6] ;

	buf [0] = (uint8_t) ( ( rxTime >> 16 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( ( rxTime >> 8 ) & 0xFF ) ;
	buf [2] = (uint8_t) ( rxTime & 0xFF ) ;
	buf [3] = (uint8_t) ( ( sleepTime >> 16 ) & 0xFF ) ;
	buf [4] = (uint8_t) ( ( sleepTime >> 8 ) & 0xFF ) ;
	buf [5] = (uint8_t) ( sleepTime & 0xFF ) ;
	WriteCommand ( radio, RADIO_SET_RXDUTYCYCLE, buf, 6 ) ;
	radio->OperatingMode = MODE_RX_DC ;
}

void SetCad ( sx126x_t* radio )
{
	WriteCommand ( radio, RADIO_SET_CAD, 0, 0 ) ;
	radio->OperatingMode = MODE_CAD ;
}

void SetTxContinuousWave ( sx126x_t* radio )
{
#ifdef ADV_DEBUG
	printf("SetTxContinuousWave ") ;
#endif
	WriteCommand ( radio, RADIO_SET_TXCONTINUOUSWAVE, 0, 0 ) ;
}

void SetTxInfinitePreamble ( sx126x_t* radio )
{
#ifdef ADV_DEBUG
	printf("SetTxContinuousPreamble ") ;
#endif
	WriteCommand ( radio, RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 ) ;
}

void SetStopRxTimerOnPreambleDetect ( sx126x_t* radio, uint8_t enable )
{
	WriteCommand ( radio, RADIO_SET_STOPRXTIMERONPREAMBLE, (uint8_t*) &enable,
			1 ) ;
}

void SetLoRaSymbNumTimeout ( sx126x_t* radio, uint8_t SymbNum )
{
	WriteCommand ( radio, RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1 ) ;
}

void SetRegulatorMode ( sx126x_t* radio, RadioRegulatorMode_t mode )
{
#ifdef ADV_DEBUG
	printf("SetRegulatorMode ") ;
#endif
	WriteCommand ( radio, RADIO_SET_REGULATORMODE, (uint8_t*) &mode, 1 ) ;
}

void Calibrate ( sx126x_t* radio, CalibrationParams_t calibParam )
{
	WriteCommand ( radio, RADIO_CALIBRATE, &calibParam.Value, 1 ) ;
}

void CalibrateImage ( sx126x_t* radio, uint32_t freq )
{
	uint8_t calFreq [2] ;

	if ( freq > 900000000 )
	{
		calFreq [0] = 0xE1 ;
		calFreq [1] = 0xE9 ;
	}
	else if ( freq > 850000000 )
	{
		calFreq [0] = 0xD7 ;
		calFreq [1] = 0xD8 ;
	}
	else if ( freq > 770000000 )
	{
		calFreq [0] = 0xC1 ;
		calFreq [1] = 0xC5 ;
	}
	else if ( freq > 460000000 )
	{
		calFreq [0] = 0x75 ;
		calFreq [1] = 0x81 ;
	}
	else if ( freq > 425000000 )
	{
		calFreq [0] = 0x6B ;
		calFreq [1] = 0x6F ;
	}
	WriteCommand ( radio, RADIO_CALIBRATEIMAGE, calFreq, 2 ) ;
}

void SetPaConfig ( sx126x_t* radio, uint8_t paDutyCycle, uint8_t HpMax,
		uint8_t deviceSel, uint8_t paLUT )
{
	uint8_t buf [4] ;

#ifdef ADV_DEBUG
	printf("SetPaConfig ") ;
#endif

	buf [0] = paDutyCycle ;
	buf [1] = HpMax ;
	buf [2] = deviceSel ;
	buf [3] = paLUT ;
	WriteCommand ( radio, RADIO_SET_PACONFIG, buf, 4 ) ;
}

void SetRxTxFallbackMode ( sx126x_t* radio, uint8_t fallbackMode )
{
	WriteCommand ( radio, RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1 ) ;
}

void SetDioIrqParams ( sx126x_t* radio, uint16_t irqMask, uint16_t dio1Mask,
		uint16_t dio2Mask, uint16_t dio3Mask )
{
	uint8_t buf [8] ;

#ifdef ADV_DEBUG
	printf("SetDioIrqParams ") ;
#endif

	buf [0] = (uint8_t) ( ( irqMask >> 8 ) & 0x00FF ) ;
	buf [1] = (uint8_t) ( irqMask & 0x00FF ) ;
	buf [2] = (uint8_t) ( ( dio1Mask >> 8 ) & 0x00FF ) ;
	buf [3] = (uint8_t) ( dio1Mask & 0x00FF ) ;
	buf [4] = (uint8_t) ( ( dio2Mask >> 8 ) & 0x00FF ) ;
	buf [5] = (uint8_t) ( dio2Mask & 0x00FF ) ;
	buf [6] = (uint8_t) ( ( dio3Mask >> 8 ) & 0x00FF ) ;
	buf [7] = (uint8_t) ( dio3Mask & 0x00FF ) ;
	WriteCommand ( radio, RADIO_CFG_DIOIRQ, buf, 8 ) ;
}

uint16_t GetIrqStatus ( sx126x_t* radio )
{
	uint8_t irqStatus [2] ;

	ReadCommand ( radio, RADIO_GET_IRQSTATUS, irqStatus, 2 ) ;
	return ( irqStatus [0] << 8 ) | irqStatus [1] ;
}

void SetDio2AsRfSwitchCtrl ( sx126x_t* radio, uint8_t enable )
{
#ifdef ADV_DEBUG
	printf("SetDio2AsRfSwitchCtrl ") ;
#endif
	WriteCommand ( radio, RADIO_SET_RFSWITCHMODE, &enable, 1 ) ;
}

void SetDio3AsTcxoCtrl ( sx126x_t* radio, RadioTcxoCtrlVoltage_t tcxoVoltage,
		uint32_t timeout )
{
	uint8_t buf [4] ;

	buf [0] = tcxoVoltage & 0x07 ;
	buf [1] = (uint8_t) ( ( timeout >> 16 ) & 0xFF ) ;
	buf [2] = (uint8_t) ( ( timeout >> 8 ) & 0xFF ) ;
	buf [3] = (uint8_t) ( timeout & 0xFF ) ;

	WriteCommand ( radio, RADIO_SET_TCXOMODE, buf, 4 ) ;
}

void SetRfFrequency ( sx126x_t* radio, uint32_t frequency )
{
	uint8_t buf [4] ;
	uint32_t freq = 0 ;

#ifdef ADV_DEBUG
	printf("SetRfFrequency ") ;
#endif

	if ( ImageCalibrated == FALSE )
	{
		CalibrateImage ( radio, frequency ) ;
		ImageCalibrated = TRUE ;
	}

	freq = (uint32_t) ( (double) frequency / (double) FREQ_STEP ) ;
	buf [0] = (uint8_t) ( ( freq >> 24 ) & 0xFF ) ;
	buf [1] = (uint8_t) ( ( freq >> 16 ) & 0xFF ) ;
	buf [2] = (uint8_t) ( ( freq >> 8 ) & 0xFF ) ;
	buf [3] = (uint8_t) ( freq & 0xFF ) ;
	WriteCommand ( radio, RADIO_SET_RFFREQUENCY, buf, 4 ) ;
}

void SetPacketType ( sx126x_t* radio, RadioPacketTypes_t packetType )
{
#ifdef ADV_DEBUG
	printf("SetPacketType ") ;
#endif

	// Save packet type internally to avoid questioning the radio
	radio->PacketType = packetType ;
	WriteCommand ( radio, RADIO_SET_PACKETTYPE, (uint8_t*) &packetType, 1 ) ;
}

RadioPacketTypes_t GetPacketType ( sx126x_t* radio )
{
	return radio->PacketType ;
}

void SetTxParams ( sx126x_t* radio, int8_t power, RadioRampTimes_t rampTime )
{
	uint8_t buf [2] ;
//	DigitalIn OPT ( A3 ) ;

#ifdef ADV_DEBUG
	printf("SetTxParams ") ;
#endif

	if ( GetDeviceType ( radio ) == SX1261 )
	{
		if ( power == 15 )
		{
			SetPaConfig ( radio, 0x06, 0x00, 0x01, 0x01 ) ;
		}
		else
		{
			SetPaConfig ( radio, 0x04, 0x00, 0x01, 0x01 ) ;
		}
		if ( power >= 14 )
		{
			power = 14 ;
		}
		else if ( power < -3 )
		{
			power = -3 ;
		}
		WriteReg ( radio, REG_OCP, 0x18 ) ; // current max is 80 mA for the whole device
	}
	else // sx1262 or sx1268
	{
		SetPaConfig ( radio, 0x04, 0x07, 0x00, 0x01 ) ;
		if ( power > 22 )
		{
			power = 22 ;
		}
		else if ( power < -3 )
		{
			power = -3 ;
		}
		WriteReg ( radio, REG_OCP, 0x38 ) ; // current max 160mA for the whole device
	}
	buf [0] = power ;
	if ( /*OPT ==*/0 )
	{
		if ( (uint8_t) rampTime < RADIO_RAMP_200_US )
		{
			buf [1] = RADIO_RAMP_200_US ;
		}
		else
		{
			buf [1] = (uint8_t) rampTime ;
		}
	}
	else
	{
		buf [1] = (uint8_t) rampTime ;
	}
	WriteCommand ( radio, RADIO_SET_TXPARAMS, buf, 2 ) ;
}

void SetModulationParams ( sx126x_t* radio,
		ModulationParams_t *modulationParams )
{
	uint8_t n ;
	uint32_t tempVal = 0 ;
	uint8_t buf [8] =
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ;

#ifdef ADV_DEBUG
	printf("SetModulationParams ") ;
#endif

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if ( radio->PacketType != modulationParams->PacketType )
	{
		SetPacketType ( radio, modulationParams->PacketType ) ;
	}

	switch ( modulationParams->PacketType )
	{
	case PACKET_TYPE_GFSK :
		n = 8 ;
		tempVal = (uint32_t) ( 32
				* ( (double) XTAL_FREQ
						/ (double) modulationParams->Params.Gfsk.BitRate ) ) ;
		buf [0] = ( tempVal >> 16 ) & 0xFF ;
		buf [1] = ( tempVal >> 8 ) & 0xFF ;
		buf [2] = tempVal & 0xFF ;
		buf [3] = modulationParams->Params.Gfsk.ModulationShaping ;
		buf [4] = modulationParams->Params.Gfsk.Bandwidth ;
		tempVal = (uint32_t) ( (double) modulationParams->Params.Gfsk.Fdev
				/ (double) FREQ_STEP ) ;
		buf [5] = ( tempVal >> 16 ) & 0xFF ;
		buf [6] = ( tempVal >> 8 ) & 0xFF ;
		buf [7] = ( tempVal & 0xFF ) ;
		break ;
	case PACKET_TYPE_LORA :
		n = 4 ;
		switch ( modulationParams->Params.LoRa.Bandwidth )
		{
		case LORA_BW_500 :
			modulationParams->Params.LoRa.LowDatarateOptimize = 0x00 ;
			break ;
		case LORA_BW_250 :
			if ( modulationParams->Params.LoRa.SpreadingFactor == 12 )
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x01 ;
			}
			else
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x00 ;
			}
			break ;
		case LORA_BW_125 :
			if ( modulationParams->Params.LoRa.SpreadingFactor >= 11 )
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x01 ;
			}
			else
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x00 ;
			}
			break ;
		case LORA_BW_062 :
			if ( modulationParams->Params.LoRa.SpreadingFactor >= 10 )
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x01 ;
			}
			else
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x00 ;
			}
			break ;
		case LORA_BW_041 :
			if ( modulationParams->Params.LoRa.SpreadingFactor >= 9 )
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x01 ;
			}
			else
			{
				modulationParams->Params.LoRa.LowDatarateOptimize = 0x00 ;
			}
			break ;
		case LORA_BW_031 :
		case LORA_BW_020 :
		case LORA_BW_015 :
		case LORA_BW_010 :
		case LORA_BW_007 :
			modulationParams->Params.LoRa.LowDatarateOptimize = 0x01 ;
			break ;
		default :
			break ;
		}
		buf [0] = modulationParams->Params.LoRa.SpreadingFactor ;
		buf [1] = modulationParams->Params.LoRa.Bandwidth ;
		buf [2] = modulationParams->Params.LoRa.CodingRate ;
		buf [3] = modulationParams->Params.LoRa.LowDatarateOptimize ;
		break ;
	default :
	case PACKET_TYPE_NONE :
		return ;
	}
	WriteCommand ( radio, RADIO_SET_MODULATIONPARAMS, buf, n ) ;
}

void SetPacketParams ( sx126x_t* radio, PacketParams_t *packetParams )
{
	uint8_t n ;
	uint8_t crcVal = 0 ;
	uint8_t buf [9] =
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ;

#ifdef ADV_DEBUG
	printf("SetPacketParams ") ;
#endif

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if ( radio->PacketType != packetParams->PacketType )
	{
		SetPacketType ( radio, packetParams->PacketType ) ;
	}

	switch ( packetParams->PacketType )
	{
	case PACKET_TYPE_GFSK :
		if ( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM )
		{
			SetCrcSeed ( radio, CRC_IBM_SEED ) ;
			SetCrcPolynomial ( radio, CRC_POLYNOMIAL_IBM ) ;
			crcVal = RADIO_CRC_2_BYTES ;
		}
		else if ( packetParams->Params.Gfsk.CrcLength
				== RADIO_CRC_2_BYTES_CCIT )
		{
			SetCrcSeed ( radio, CRC_CCITT_SEED ) ;
			SetCrcPolynomial ( radio, CRC_POLYNOMIAL_CCITT ) ;
			crcVal = RADIO_CRC_2_BYTES_INV ;
		}
		else
		{
			crcVal = packetParams->Params.Gfsk.CrcLength ;
		}
		n = 9 ;
		// convert preamble length from byte to bit
		packetParams->Params.Gfsk.PreambleLength =
				packetParams->Params.Gfsk.PreambleLength << 3 ;

		buf [0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF ;
		buf [1] = packetParams->Params.Gfsk.PreambleLength ;
		buf [2] = packetParams->Params.Gfsk.PreambleMinDetect ;
		buf [3] = ( packetParams->Params.Gfsk.SyncWordLength << 3 ) ; // convert from byte to bit
		buf [4] = packetParams->Params.Gfsk.AddrComp ;
		buf [5] = packetParams->Params.Gfsk.HeaderType ;
		buf [6] = packetParams->Params.Gfsk.PayloadLength ;
		buf [7] = crcVal ;
		buf [8] = packetParams->Params.Gfsk.DcFree ;
		break ;
	case PACKET_TYPE_LORA :
		n = 6 ;
		buf [0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF ;
		buf [1] = packetParams->Params.LoRa.PreambleLength ;
		buf [2] = packetParams->Params.LoRa.HeaderType ;
		buf [3] = packetParams->Params.LoRa.PayloadLength ;
		buf [4] = packetParams->Params.LoRa.CrcMode ;
		buf [5] = packetParams->Params.LoRa.InvertIQ ;
		break ;
	default :
	case PACKET_TYPE_NONE :
		return ;
	}
	WriteCommand ( radio, RADIO_SET_PACKETPARAMS, buf, n ) ;
}

void SetCadParams ( sx126x_t* radio, RadioLoRaCadSymbols_t cadSymbolNum,
		uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode,
		uint32_t cadTimeout )
{
	uint8_t buf [7] ;

	buf [0] = (uint8_t) cadSymbolNum ;
	buf [1] = cadDetPeak ;
	buf [2] = cadDetMin ;
	buf [3] = (uint8_t) cadExitMode ;
	buf [4] = (uint8_t) ( ( cadTimeout >> 16 ) & 0xFF ) ;
	buf [5] = (uint8_t) ( ( cadTimeout >> 8 ) & 0xFF ) ;
	buf [6] = (uint8_t) ( cadTimeout & 0xFF ) ;
	WriteCommand ( radio, RADIO_SET_CADPARAMS, buf, 7 ) ;
	radio->OperatingMode = MODE_CAD ;
}

void SetBufferBaseAddresses ( sx126x_t* radio, uint8_t txBaseAddress,
		uint8_t rxBaseAddress )
{
	uint8_t buf [2] ;

#ifdef ADV_DEBUG
	printf("SetBufferBaseAddresses ") ;
#endif

	buf [0] = txBaseAddress ;
	buf [1] = rxBaseAddress ;
	WriteCommand ( radio, RADIO_SET_BUFFERBASEADDRESS, buf, 2 ) ;
}

RadioStatus_t GetStatus ( sx126x_t* radio )
{
	uint8_t stat = 0 ;
	RadioStatus_t status ;

	ReadCommand ( radio, RADIO_GET_STATUS, (uint8_t *) &stat, 1 ) ;
	status.Value = stat ;
	return status ;
}

int8_t GetRssiInst ( sx126x_t* radio )
{
	uint8_t rssi ;

	ReadCommand ( radio, RADIO_GET_RSSIINST, (uint8_t*) &rssi, 1 ) ;
	return ( - ( rssi / 2 ) ) ;
}

void GetRxBufferStatus ( sx126x_t* radio, uint8_t *payloadLength,
		uint8_t *rxStartBufferPointer )
{
	uint8_t status [2] ;

	ReadCommand ( radio, RADIO_GET_RXBUFFERSTATUS, status, 2 ) ;

	// In case of LORA fixed header, the payloadLength is obtained by reading
	// the register REG_LR_PAYLOADLENGTH
	if ( ( GetPacketType ( radio ) == PACKET_TYPE_LORA )
			&& ( ReadReg ( radio, REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
	{
		*payloadLength = ReadReg ( radio, REG_LR_PAYLOADLENGTH ) ;
	}
	else
	{
		*payloadLength = status [0] ;
	}
	*rxStartBufferPointer = status [1] ;
}

void GetPacketStatus ( sx126x_t* radio, PacketStatus_t *pktStatus )
{
	uint8_t status [3] ;

	ReadCommand ( radio, RADIO_GET_PACKETSTATUS, status, 3 ) ;

	pktStatus->packetType = GetPacketType ( radio ) ;
	switch ( pktStatus->packetType )
	{
	case PACKET_TYPE_GFSK :
		pktStatus->Params.Gfsk.RxStatus = status [0] ;
		pktStatus->Params.Gfsk.RssiSync = -status [1] / 2 ;
		pktStatus->Params.Gfsk.RssiAvg = -status [2] / 2 ;
		pktStatus->Params.Gfsk.FreqError = 0 ;
		break ;

	case PACKET_TYPE_LORA :
		pktStatus->Params.LoRa.RssiPkt = -status [0] / 2 ;
		( status [1] < 128 ) ?
				( pktStatus->Params.LoRa.SnrPkt = status [1] / 4 ) :
				( pktStatus->Params.LoRa.SnrPkt = ( ( status [1] - 256 ) / 4 ) ) ;
		pktStatus->Params.LoRa.SignalRssiPkt = -status [2] / 2 ;
		pktStatus->Params.LoRa.FreqError = FrequencyError ;
		break ;

	default :
	case PACKET_TYPE_NONE :
		// In that specific case, we set everything in the pktStatus to zeros
		// and reset the packet type accordingly
		memset ( pktStatus, 0, sizeof(PacketStatus_t) ) ;
		pktStatus->packetType = PACKET_TYPE_NONE ;
		break ;
	}
}

RadioError_t GetDeviceErrors ( sx126x_t* radio )
{
	RadioError_t error ;

	ReadCommand ( radio, RADIO_GET_ERROR, (uint8_t *) &error, 2 ) ;
	return error ;
}

void ClearIrqStatus ( sx126x_t* radio, uint16_t irq )
{
	uint8_t buf [2] ;
#ifdef ADV_DEBUG
	printf("ClearIrqStatus ") ;
#endif
	buf [0] = (uint8_t) ( ( (uint16_t) irq >> 8 ) & 0x00FF ) ;
	buf [1] = (uint8_t) ( (uint16_t) irq & 0x00FF ) ;
	WriteCommand ( radio, RADIO_CLR_IRQSTATUS, buf, 2 ) ;
}

void SetPollingMode ( sx126x_t* radio )
{
	radio->PollingMode = TRUE ;
}

void SetInterruptMode ( sx126x_t* radio )
{
	radio->PollingMode = FALSE ;
}

void OnDioIrq ( sx126x_t* radio )
{
	/*
	 * When polling mode is activated, it is up to the application to call
	 * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
	 * on radio interrupt.
	 */
	if ( radio->PollingMode == TRUE )
	{
		radio->IrqState = TRUE ;
	}
	else
	{
		ProcessIrqs ( radio ) ;
	}
}

void ProcessIrqs ( sx126x_t* radio )
{
	if ( radio->PollingMode == TRUE )
	{
		if ( radio->IrqState == TRUE )
		{
			radio->set_interrupts ( FALSE ) ;
			radio->IrqState = FALSE ;
			radio->set_interrupts ( TRUE ) ;
		}
		else
		{
			return ;
		}
	}

	uint16_t irqRegs = GetIrqStatus ( radio ) ;
	ClearIrqStatus ( radio, IRQ_RADIO_ALL ) ;

#ifdef ADV_DEBUG
	printf("0x%04x\n\r", irqRegs ) ;
#endif

	if ( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
	{
		// LoRa Only
		FrequencyError = 0x000000
				| ( ( 0x0F & ReadReg ( radio, REG_FREQUENCY_ERRORBASEADDR ) )
						<< 16 ) ;
		FrequencyError = FrequencyError
				| ( ReadReg ( radio, REG_FREQUENCY_ERRORBASEADDR + 1 ) << 8 ) ;
		FrequencyError = FrequencyError
				| ( ReadReg ( radio, REG_FREQUENCY_ERRORBASEADDR + 2 ) ) ;
	}

	if ( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
	{
		if ( radio->callbacks->txDone != NULL )
		{
			radio->callbacks->txDone () ;
		}
	}

	if ( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
	{
		if ( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
		{
			if ( radio->callbacks->rxError != NULL )
			{
				radio->callbacks->rxError ( IRQ_CRC_ERROR_CODE ) ;
			}
		}
		else
		{
			if ( radio->callbacks->rxDone != NULL )
			{
				radio->callbacks->rxDone () ;
			}
		}
	}

	if ( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
	{
		if ( radio->callbacks->cadDone != NULL )
		{
			radio->callbacks->cadDone (
					( irqRegs & IRQ_CAD_ACTIVITY_DETECTED )
							== IRQ_CAD_ACTIVITY_DETECTED ) ;
		}
	}

	if ( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
	{
		if ( ( radio->callbacks->txTimeout != NULL )
				&& ( radio->OperatingMode == MODE_TX ) )
		{
			radio->callbacks->txTimeout () ;
		}
		else if ( ( radio->callbacks->rxTimeout != NULL )
				&& ( radio->OperatingMode == MODE_RX ) )
		{
			radio->callbacks->rxTimeout () ;
		}
		else
		{
			assert_param( FAIL ) ;
		}
	}

	/*
	 //IRQ_PREAMBLE_DETECTED                   = 0x0004,
	 if( irqRegs & IRQ_PREAMBLE_DETECTED )
	 {
	 if( rxPblSyncWordHeader != NULL )
	 {
	 rxPblSyncWordHeader( IRQ_PBL_DETECT_CODE);

	 }
	 }

	 //IRQ_SYNCWORD_VALID                      = 0x0008,
	 if( irqRegs & IRQ_SYNCWORD_VALID )
	 {
	 if( rxPblSyncWordHeader != NULL )
	 {
	 rxPblSyncWordHeader( IRQ_SYNCWORD_VALID_CODE  );
	 }
	 }

	 //IRQ_HEADER_VALID                        = 0x0010,
	 if ( irqRegs & IRQ_HEADER_VALID )
	 {
	 if( rxPblSyncWordHeader != NULL )
	 {
	 rxPblSyncWordHeader( IRQ_HEADER_VALID_CODE );
	 }
	 }

	 //IRQ_HEADER_ERROR                        = 0x0020,
	 if( irqRegs & IRQ_HEADER_ERROR )
	 {
	 if( rxError != NULL )
	 {
	 rxError( IRQ_HEADER_ERROR_CODE );
	 }
	 }
	 */
}

//SX126xHal ( PinName mosi, PinName miso, PinName sclk, PinName nss, PinName busy,
//		PinName dio1, PinName dio2, PinName dio3, PinName rst, PinName freqSel,
//		PinName deviceSelect, PinName antSwPower, RadioCallbacks_t *callbacks )
//:   SX126x( callbacks ),
//            RadioNss( nss ),
//            RadioReset( rst ),
//            BUSY( busy ),
//            FreqSelect( freqSel ),
//            DeviceSelect( deviceSelect ),
//            antSwitchPower( antSwPower )
//{
//    CreateDioPin( dio1, DIO1 ) ;CreateDioPin( dio2, DIO2 ) ;CreateDioPin( dio3,
//		DIO3 ) ;
//RadioSpi = new SPI( mosi, miso, sclk ) ;
//
//RadioNss = 1 ;
//RadioReset = 1 ;
//}

//void IoIrqInit ( DioIrqHandler irqHandler )
//{
//	assert_param( RadioSpi != NULL ) ;
//	if ( RadioSpi != NULL )
//	{
//		SpiInit () ;
//	}
//
//	BUSY.mode ( PullNone ) ;
//	DioAssignCallback( DIO1, PullNone, irqHandler ) ;
////    DioAssignCallback( DIO2, PullNone, irqHandler );
////    DioAssignCallback( DIO3, PullNone, irqHandler );
//
//}

void Reset ( sx126x_t* radio )
{
	radio->set_interrupts ( FALSE ) ;
	radio->wait_ms ( 20 ) ;
	radio->IO_callbacks->setRadioRESET ( FALSE ) ;
	radio->wait_ms ( 50 ) ;
	radio->IO_callbacks->setRadioRESET ( TRUE ) ; // Using the internal pull-up
	radio->wait_ms ( 20 ) ;
	radio->set_interrupts ( TRUE ) ;
}

void Wakeup ( sx126x_t* radio )
{
	radio->set_interrupts ( FALSE ) ;

//Don't wait for BUSY here

	uint8_t temp = RADIO_GET_STATUS ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = 0 ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;

// Wait for chip to be ready.

	WaitOnBusy ( radio ) ;

	radio->set_interrupts ( TRUE ) ;

	AntSwOn ( radio ) ;
}

void WriteCommand ( sx126x_t* radio, RadioCommands_t command, uint8_t *buffer,
		uint16_t size )
{
#ifdef ADV_DEBUG
	printf("cmd: 0x%02x", command ) ;
	for( uint8_t i = 0 ; i < size ; i++ )
	{
		printf("-%02x", buffer[i] ) ;
	}
	printf("\n\r") ;
#endif

	WaitOnBusy ( radio ) ;

	uint8_t temp = command ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->spi_Tx ( buffer, size ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;
	WaitOnCounter () ;
}

void ReadCommand ( sx126x_t* radio, RadioCommands_t command, uint8_t *buffer,
		uint16_t size )
{
	WaitOnBusy ( radio ) ;

	uint8_t temp = command ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = 0 ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->spi_Rx ( buffer, size ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;
}

void WriteRegister ( sx126x_t* radio, uint16_t address, uint8_t *buffer,
		uint16_t size )
{
	WaitOnBusy ( radio ) ;

	uint8_t temp = RADIO_WRITE_REGISTER ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = ( address & 0xFF00 ) >> 8 ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = address & 0x00FF ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->spi_Tx ( buffer, size ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;
}

void WriteReg ( sx126x_t* radio, uint16_t address, uint8_t value )
{
	WriteRegister ( radio, address, &value, 1 ) ;
}

void ReadRegister ( sx126x_t* radio, uint16_t address, uint8_t *buffer,
		uint16_t size )
{
	WaitOnBusy ( radio ) ;

	uint8_t temp = RADIO_READ_REGISTER ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = ( address & 0xFF00 ) >> 8 ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = address & 0x00FF ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->spi_Rx ( buffer, size ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;
}

uint8_t ReadReg ( sx126x_t* radio, uint16_t address )
{
	uint8_t data ;

	ReadRegister ( radio, address, &data, 1 ) ;
	return data ;
}

void WriteBuffer ( sx126x_t* radio, uint8_t offset, uint8_t *buffer,
		uint8_t size )
{
	WaitOnBusy ( radio ) ;

	uint8_t temp = RADIO_WRITE_BUFFER ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = offset ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->spi_Tx ( buffer, size ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;
}

void ReadBuffer ( sx126x_t* radio, uint8_t offset, uint8_t *buffer,
		uint8_t size )
{
	WaitOnBusy ( radio ) ;

	uint8_t temp = RADIO_READ_BUFFER ;
	radio->IO_callbacks->setRadioNSS ( FALSE ) ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = offset ;
	radio->spi_Tx ( &temp, 1 ) ;
	temp = 0 ;
	radio->spi_Tx ( &temp, 1 ) ;
	radio->spi_Rx ( buffer, size ) ;
	radio->IO_callbacks->setRadioNSS ( TRUE ) ;
}

//uint8_t GetDioStatus ( sx126x_t* radio )
//{
//	return ( *DIO3 << 3 ) | ( *DIO2 << 2 ) | ( *DIO1 << 1 ) | ( BUSY << 0 ) ;
//}

uint8_t GetDeviceType ( sx126x_t* radio )
{
	uint16_t val = 0x2000 ;
//	val = DeviceSelect.read_u16 () ;

	if ( val <= 0x2000 )
	{
		return ( SX1262 ) ;
	}
	else if ( val <= 0xA000 )
	{
		return ( SX1268 ) ;
	}
	else
	{
		return ( SX1261 ) ;
	}
}

uint8_t GetFreqSelect ( sx126x_t* radio )
{
	uint16_t val = 0 ;
//	val = FreqSelect.read_u16 () ;

	if ( val < 100 )
	{
		return ( MATCHING_FREQ_915 ) ;
	}
	else if ( val <= 0x3000 )
	{
		return ( MATCHING_FREQ_780 ) ;
	}
	else if ( val <= 0x4900 )       // 0x4724
	{
		return ( MATCHING_FREQ_490 ) ;
	}
	else if ( val <= 1 )
	{
		return ( MATCHING_FREQ_434 ) ;
	}
	else if ( val <= 1 )
	{
		return ( MATCHING_FREQ_280 ) ;
	}
	else if ( val <= 0xF000 )
	{
		return ( MATCHING_FREQ_169 ) ;
	}
	else
	{
		return ( MATCHING_FREQ_868 ) ;
	}
}

void AntSwOn ( sx126x_t* radio )
{
	radio->IO_callbacks->setRadioAntSwitchPower ( TRUE ) ;
}

void AntSwOff ( sx126x_t* radio )
{
	radio->IO_callbacks->setRadioAntSwitchPower ( FALSE ) ;
}
/* Implementations End -------------------------------------------------------*/
