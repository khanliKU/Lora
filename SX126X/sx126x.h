/**
 ******************************************************************************
 * @file		: sx126x.h
 * @author		: Kutlay Hanl�
 * @email		: khanli@ku.edu.tr
 * @date		: 20 Mar 2020
 * @brief		: TODO fill in
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Delta V nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef SX126X_H_
#define SX126X_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
/* Includes END --------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#ifndef TRUE
#define TRUE (1==1)
#define FALSE !TRUE
#endif

#ifndef NULL
#define NULL 0
#endif

/*!
 * \brief Provides the frequency of the chip running on the radio and the frequency step
 *
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define XTAL_FREQ                                   32000000
#define FREQ_DIV                                    33554432
#define FREQ_STEP                                   0.95367431640625 // ( ( double )( XTAL_FREQ / ( double )FREQ_DIV ) )
#define FREQ_ERR                                    0.47683715820312

/*!
 * \brief List of devices supported by this driver
 */
#define SX1261  0
#define SX1262  1
#define SX1268  2

/*!
 * \brief List of matching supported by the sx126x
 */
#define MATCHING_FREQ_915                           0
#define MATCHING_FREQ_780                           1
#define MATCHING_FREQ_490                           2
#define MATCHING_FREQ_434                           3
#define MATCHING_FREQ_280                           4
#define MATCHING_FREQ_169                           5
#define MATCHING_FREQ_868                           6

/*!
 * \brief Compensation delay for SetAutoTx/Rx functions in 15.625 microseconds
 */
#define AUTO_RX_TX_OFFSET                           2

/*!
 * \brief LFSR initial value to compute IBM type CRC
 */
#define CRC_IBM_SEED                                0xFFFF

/*!
 * \brief LFSR initial value to compute CCIT type CRC
 */
#define CRC_CCITT_SEED                              0x1D0F

/*!
 * \brief Polynomial used to compute IBM CRC
 */
#define CRC_POLYNOMIAL_IBM                          0x8005

/*!
 * \brief Polynomial used to compute CCIT CRC
 */
#define CRC_POLYNOMIAL_CCITT                        0x1021

/*!
 * \brief The address of the register holding the first byte defining the CRC seed
 *
 */
#define REG_LR_CRCSEEDBASEADDR                      0x06BC

/*!
 * \brief The address of the register holding the first byte defining the CRC polynomial
 */
#define REG_LR_CRCPOLYBASEADDR                      0x06BE

/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 */
#define REG_LR_WHITSEEDBASEADDR_MSB                 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB                 0x06B9

/*!
 * \brief The address of the register holding the packet configuration
 */
#define REG_LR_PACKETPARAMS                         0x0704

/*!
 * \brief The address of the register holding the payload size
 */
#define REG_LR_PAYLOADLENGTH                        0x0702

/*!
 * \brief The addresses of the registers holding SyncWords values
 */
#define REG_LR_SYNCWORDBASEADDRESS                  0x06C0

/*!
 * \brief The addresses of the register holding LoRa Modem SyncWord value
 */
#define REG_LR_SYNCWORD                             0x0740

/*!
 * Syncword for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x1424

/*!
 * Syncword for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x3444

/*!
 * The address of the register giving a 4 bytes random number
 */
#define RANDOM_NUMBER_GENERATORBASEADDR             0x0819

/*!
 * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
 */
#define REG_RX_GAIN                                 0x08AC

/*!
 * The address of the register holding frequency error indication
 */
#define REG_FREQUENCY_ERRORBASEADDR                 0x076B

/*!
 * Change the value on the device internal trimming capacitor
 */
#define REG_XTA_TRIM                                0x0911

/*!
 * Set the current max value in the over current protection
 */
#define REG_OCP                                     0x08E7
/* Private define END --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported macro END --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/*!
 * \brief Structure describing the radio status
 */
typedef union RadioStatus_u
{
	uint8_t Value ;
	struct
	{   //bit order is lsb -> msb
		uint8_t Reserved :1 ;  //!< Reserved
		uint8_t CmdStatus :3 ;  //!< Command status
		uint8_t ChipMode :3 ;  //!< Chip mode
		uint8_t CpuBusy :1 ;  //!< Flag for CPU radio busy
	} Fields ;
} RadioStatus_t ;

/*!
 * \brief Structure describing the error codes for callback functions
 */
typedef enum
{
	IRQ_HEADER_ERROR_CODE = 0x01,
	IRQ_SYNCWORD_ERROR_CODE = 0x02,
	IRQ_CRC_ERROR_CODE = 0x04,
} IrqErrorCode_t ;

enum IrqPblSyncHeaderCode_t
{
	IRQ_PBL_DETECT_CODE = 0x01,
	IRQ_SYNCWORD_VALID_CODE = 0x02,
	IRQ_HEADER_VALID_CODE = 0x04,
} ;

enum IrqTimeoutCode_t
{
	IRQ_RX_TIMEOUT = 0x00, IRQ_TX_TIMEOUT = 0x01, IRQ_XYZ = 0xFF,
} ;

/*!
 * \brief The radio command opcodes
 */
typedef enum RadioCommands_e RadioCommands_t ;

/*!
 * \brief The radio callbacks structure
 * Holds function pointers to be called on radio interrupts
 */
typedef struct
{
	void (*txDone) ( void ) ; //!< Pointer to a function run on successful transmission
	void (*rxDone) ( void ) ; //!< Pointer to a function run on successful reception
	void (*rxPreambleDetect) ( void ) ; //!< Pointer to a function run on successful Preamble detection
	void (*rxSyncWordDone) ( void ) ; //!< Pointer to a function run on successful SyncWord reception
	void (*rxHeaderDone) ( void ) ; //!< Pointer to a function run on successful Header reception
	void (*txTimeout) ( void ) ; //!< Pointer to a function run on transmission timeout
	void (*rxTimeout) ( void ) ; //!< Pointer to a function run on reception timeout
	void (*rxError) ( IrqErrorCode_t errCode ) ; //!< Pointer to a function run on reception error
	void (*cadDone) ( uint8_t cadFlag ) ; //!< Pointer to a function run on channel activity detected
} RadioCallbacks_t ;

/*!
 * \brief The radio IO callbacks structure
 * Holds function pointers to be called on radio IO
 */
typedef struct
{
	void (*setRadioNSS) ( uint8_t value ) ; //!< The pin connected to Radio chip select (active low)
	void (*setRadioRESET) ( uint8_t value ) ; //!< The reset pin connected to the radio
	uint8_t (*getRadioBUSY) ( void ) ; //!< The pin connected to BUSY
	void (*setRadioAntSwitchPower) ( uint8_t value ) ; //!< The pin connected to the RF Switch Power
} RadioIOCallbacks_t ;

/*!
 * \brief Represents the states of the radio
 */
typedef enum
{
	RF_IDLE = 0x00,         //!< The radio is idle
	RF_RX_RUNNING,                          //!< The radio is in reception state
	RF_TX_RUNNING,                       //!< The radio is in transmission state
	RF_CAD,                  //!< The radio is doing channel activity detection
} RadioStates_t ;

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
	MODE_SLEEP = 0x00,         //! The radio is in sleep mode
	MODE_STDBY_RC,           //! The radio is in standby mode with RC oscillator
	MODE_STDBY_XOSC,       //! The radio is in standby mode with XOSC oscillator
	MODE_FS,                        //! The radio is in frequency synthesis mode
	MODE_TX,                                   //! The radio is in transmit mode
	MODE_RX,                                    //! The radio is in receive mode
	MODE_RX_DC,                      //! The radio is in receive duty cycle mode
	MODE_CAD                 //! The radio is in channel activity detection mode
} RadioOperatingModes_t ;

/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
	STDBY_RC = 0x00, STDBY_XOSC = 0x01,
} RadioStandbyModes_t ;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
	USE_LDO = 0x00, // default
	USE_DCDC = 0x01,
} RadioRegulatorMode_t ;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
	PACKET_TYPE_GFSK = 0x00, PACKET_TYPE_LORA = 0x01, PACKET_TYPE_NONE = 0x0F,
} RadioPacketTypes_t ;

/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
	RADIO_RAMP_10_US = 0x00,
	RADIO_RAMP_20_US = 0x01,
	RADIO_RAMP_40_US = 0x02,
	RADIO_RAMP_80_US = 0x03,
	RADIO_RAMP_200_US = 0x04,
	RADIO_RAMP_800_US = 0x05,
	RADIO_RAMP_1700_US = 0x06,
	RADIO_RAMP_3400_US = 0x07,
} RadioRampTimes_t ;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
	LORA_CAD_01_SYMBOL = 0x00,
	LORA_CAD_02_SYMBOL = 0x01,
	LORA_CAD_04_SYMBOL = 0x02,
	LORA_CAD_08_SYMBOL = 0x03,
	LORA_CAD_16_SYMBOL = 0x04,
} RadioLoRaCadSymbols_t ;

/*!
 * \brief Represents the Channel Activity Detection actions after the CAD operation is finished
 */
typedef enum
{
	LORA_CAD_ONLY = 0x00, LORA_CAD_RX = 0x01, LORA_CAD_LBT = 0x10,
} RadioCadExitModes_t ;

/*!
 * \brief Represents the modulation shaping parameter
 */
typedef enum
{
	MOD_SHAPING_OFF = 0x00,
	MOD_SHAPING_G_BT_03 = 0x08,
	MOD_SHAPING_G_BT_05 = 0x09,
	MOD_SHAPING_G_BT_07 = 0x0A,
	MOD_SHAPING_G_BT_1 = 0x0B,
} RadioModShapings_t ;

/*!
 * \brief Represents the modulation shaping parameter
 */
typedef enum
{
	RX_BW_4800 = 0x1F,
	RX_BW_5800 = 0x17,
	RX_BW_7300 = 0x0F,
	RX_BW_9700 = 0x1E,
	RX_BW_11700 = 0x16,
	RX_BW_14600 = 0x0E,
	RX_BW_19500 = 0x1D,
	RX_BW_23400 = 0x15,
	RX_BW_29300 = 0x0D,
	RX_BW_39000 = 0x1C,
	RX_BW_46900 = 0x14,
	RX_BW_58600 = 0x0C,
	RX_BW_78200 = 0x1B,
	RX_BW_93800 = 0x13,
	RX_BW_117300 = 0x0B,
	RX_BW_156200 = 0x1A,
	RX_BW_187200 = 0x12,
	RX_BW_234300 = 0x0A,
	RX_BW_312000 = 0x19,
	RX_BW_373600 = 0x11,
	RX_BW_467000 = 0x09,
} RadioRxBandwidth_t ;

/*!
 * \brief Represents the possible spreading factor values in LoRa packet types
 */
typedef enum
{
	LORA_SF5 = 0x05,
	LORA_SF6 = 0x06,
	LORA_SF7 = 0x07,
	LORA_SF8 = 0x08,
	LORA_SF9 = 0x09,
	LORA_SF10 = 0x0A,
	LORA_SF11 = 0x0B,
	LORA_SF12 = 0x0C,
} RadioLoRaSpreadingFactors_t ;

/*!
 * \brief Represents the bandwidth values for LoRa packet type
 */
typedef enum
{
	LORA_BW_500 = 6,
	LORA_BW_250 = 5,
	LORA_BW_125 = 4,
	LORA_BW_062 = 3,
	LORA_BW_041 = 10,
	LORA_BW_031 = 2,
	LORA_BW_020 = 9,
	LORA_BW_015 = 1,
	LORA_BW_010 = 8,
	LORA_BW_007 = 0,
} RadioLoRaBandwidths_t ;

/*!
 * \brief Represents the coding rate values for LoRa packet type
 */
typedef enum
{
	LORA_CR_4_5 = 0x01,
	LORA_CR_4_6 = 0x02,
	LORA_CR_4_7 = 0x03,
	LORA_CR_4_8 = 0x04,
} RadioLoRaCodingRates_t ;

/*!
 * \brief Represents the preamble length used to detect the packet on Rx side
 */
typedef enum
{
	RADIO_PREAMBLE_DETECTOR_OFF = 0x00,       //!< Preamble detection length off
	RADIO_PREAMBLE_DETECTOR_08_BITS = 0x04, //!< Preamble detection length 8 bits
	RADIO_PREAMBLE_DETECTOR_16_BITS = 0x05, //!< Preamble detection length 16 bits
	RADIO_PREAMBLE_DETECTOR_24_BITS = 0x06, //!< Preamble detection length 24 bits
	RADIO_PREAMBLE_DETECTOR_32_BITS = 0x07, //!< Preamble detection length 32 bit
} RadioPreambleDetection_t ;

/*!
 * \brief Represents the possible combinations of SyncWord correlators activated
 */
typedef enum
{
	RADIO_ADDRESSCOMP_FILT_OFF = 0x00, //!< No correlator turned on, i.e. do not search for SyncWord
	RADIO_ADDRESSCOMP_FILT_NODE = 0x01,
	RADIO_ADDRESSCOMP_FILT_NODE_BROAD = 0x02,
} RadioAddressComp_t ;

/*!
 *  \brief Radio packet length mode
 */
typedef enum
{
	RADIO_PACKET_FIXED_LENGTH = 0x00, //!< The packet is known on both sides, no header included in the packet
	RADIO_PACKET_VARIABLE_LENGTH = 0x01, //!< The packet is on variable size, header included
} RadioPacketLengthModes_t ;

/*!
 * \brief Represents the CRC length
 */
typedef enum
{
	RADIO_CRC_OFF = 0x01,         //!< No CRC in use
	RADIO_CRC_1_BYTES = 0x00,
	RADIO_CRC_2_BYTES = 0x02,
	RADIO_CRC_1_BYTES_INV = 0x04,
	RADIO_CRC_2_BYTES_INV = 0x06,
	RADIO_CRC_2_BYTES_IBM = 0xF1,
	RADIO_CRC_2_BYTES_CCIT = 0xF2,
} RadioCrcTypes_t ;

/*!
 * \brief Radio whitening mode activated or deactivated
 */
typedef enum
{
	RADIO_DC_FREE_OFF = 0x00, RADIO_DC_FREEWHITENING = 0x01,
} RadioDcFree_t ;

/*!
 * \brief Holds the lengths mode of a LoRa packet type
 */
typedef enum
{
	LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
	LORA_PACKET_FIXED_LENGTH = 0x01, //!< The packet is known on both sides, no header included in the packet
	LORA_PACKET_EXPLICIT = LORA_PACKET_VARIABLE_LENGTH,
	LORA_PACKET_IMPLICIT = LORA_PACKET_FIXED_LENGTH,
} RadioLoRaPacketLengthsMode_t ;

/*!
 * \brief Represents the CRC mode for LoRa packet type
 */
typedef enum
{
	LORA_CRC_ON = 0x01,         //!< CRC activated
	LORA_CRC_OFF = 0x00,         //!< CRC not used
} RadioLoRaCrcModes_t ;

/*!
 * \brief Represents the IQ mode for LoRa packet type
 */
typedef enum
{
	LORA_IQ_NORMAL = 0x00, LORA_IQ_INVERTED = 0x01,
} RadioLoRaIQModes_t ;

/*!
 * \brief Represents the volatge used to control the TCXO on/off from DIO3
 */
typedef enum
{
	TCXO_CTRL_1_6V = 0x00,
	TCXO_CTRL_1_7V = 0x01,
	TCXO_CTRL_1_8V = 0x02,
	TCXO_CTRL_2_2V = 0x03,
	TCXO_CTRL_2_4V = 0x04,
	TCXO_CTRL_2_7V = 0x05,
	TCXO_CTRL_3_0V = 0x06,
	TCXO_CTRL_3_3V = 0x07,
} RadioTcxoCtrlVoltage_t ;

/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
	IRQ_RADIO_NONE = 0x0000,
	IRQ_TX_DONE = 0x0001,
	IRQ_RX_DONE = 0x0002,
	IRQ_PREAMBLE_DETECTED = 0x0004,
	IRQ_SYNCWORD_VALID = 0x0008,
	IRQ_HEADER_VALID = 0x0010,
	IRQ_HEADER_ERROR = 0x0020,
	IRQ_CRC_ERROR = 0x0040,
	IRQ_CAD_DONE = 0x0080,
	IRQ_CAD_ACTIVITY_DETECTED = 0x0100,
	IRQ_RX_TX_TIMEOUT = 0x0200,
	IRQ_RADIO_ALL = 0xFFFF,
} RadioIrqMasks_t ;

/*!
 * \brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_e
{
	RADIO_GET_STATUS = 0xC0,
	RADIO_WRITE_REGISTER = 0x0D,
	RADIO_READ_REGISTER = 0x1D,
	RADIO_WRITE_BUFFER = 0x0E,
	RADIO_READ_BUFFER = 0x1E,
	RADIO_SET_SLEEP = 0x84,
	RADIO_SET_STANDBY = 0x80,
	RADIO_SET_FS = 0xC1,
	RADIO_SET_TX = 0x83,
	RADIO_SET_RX = 0x82,
	RADIO_SET_RXDUTYCYCLE = 0x94,
	RADIO_SET_CAD = 0xC5,
	RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
	RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
	RADIO_SET_PACKETTYPE = 0x8A,
	RADIO_GET_PACKETTYPE = 0x11,
	RADIO_SET_RFFREQUENCY = 0x86,
	RADIO_SET_TXPARAMS = 0x8E,
	RADIO_SET_PACONFIG = 0x95,
	RADIO_SET_CADPARAMS = 0x88,
	RADIO_SET_BUFFERBASEADDRESS = 0x8F,
	RADIO_SET_MODULATIONPARAMS = 0x8B,
	RADIO_SET_PACKETPARAMS = 0x8C,
	RADIO_GET_RXBUFFERSTATUS = 0x13,
	RADIO_GET_PACKETSTATUS = 0x14,
	RADIO_GET_RSSIINST = 0x15,
	RADIO_GET_STATS = 0x10,
	RADIO_RESET_STATS = 0x00,
	RADIO_CFG_DIOIRQ = 0x08,
	RADIO_GET_IRQSTATUS = 0x12,
	RADIO_CLR_IRQSTATUS = 0x02,
	RADIO_CALIBRATE = 0x89,
	RADIO_CALIBRATEIMAGE = 0x98,
	RADIO_SET_REGULATORMODE = 0x96,
	RADIO_GET_ERROR = 0x17,
	RADIO_SET_TCXOMODE = 0x97,
	RADIO_SET_TXFALLBACKMODE = 0x93,
	RADIO_SET_RFSWITCHMODE = 0x9D,
	RADIO_SET_STOPRXTIMERONPREAMBLE = 0x9F,
	RADIO_SET_LORASYMBTIMEOUT = 0xA0,
} RadioCommands_t ;

/*!
 * \brief The type describing the modulation parameters for every packet types
 */
typedef struct
{
	RadioPacketTypes_t PacketType ; //!< Packet to which the modulation parameters are referring to.
	struct
	{
		struct
		{
			uint32_t BitRate ;
			uint32_t Fdev ;
			RadioModShapings_t ModulationShaping ;
			uint8_t Bandwidth ;
		} Gfsk ;
		struct
		{
			RadioLoRaSpreadingFactors_t SpreadingFactor ; //!< Spreading Factor for the LoRa modulation
			RadioLoRaBandwidths_t Bandwidth ; //!< Bandwidth for the LoRa modulation
			RadioLoRaCodingRates_t CodingRate ; //!< Coding rate for the LoRa modulation
			uint8_t LowDatarateOptimize ; //!< Indicates if the modem uses the low datarate optimization
		} LoRa ;
	} Params ;                    //!< Holds the modulation parameters structure
} ModulationParams_t ;

/*!
 * \brief The type describing the packet parameters for every packet types
 */
typedef struct
{
	RadioPacketTypes_t PacketType ; //!< Packet to which the packet parameters are referring to.
	struct
	{
		/*!
		 * \brief Holds the GFSK packet parameters
		 */
		struct
		{
			uint16_t PreambleLength ; //!< The preamble Tx length for GFSK packet type in bit
			RadioPreambleDetection_t PreambleMinDetect ; //!< The preamble Rx length minimal for GFSK packet type
			uint8_t SyncWordLength ; //!< The synchronization word length for GFSK packet type
			RadioAddressComp_t AddrComp ;    //!< Activated SyncWord correlators
			RadioPacketLengthModes_t HeaderType ; //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
			uint8_t PayloadLength ;  //!< Size of the payload in the GFSK packet
			RadioCrcTypes_t CrcLength ; //!< Size of the CRC block in the GFSK packet
			RadioDcFree_t DcFree ;
		} Gfsk ;
		/*!
		 * \brief Holds the LoRa packet parameters
		 */
		struct
		{
			uint16_t PreambleLength ; //!< The preamble length is the number of LoRa symbols in the preamble
			RadioLoRaPacketLengthsMode_t HeaderType ; //!< If the header is explicit, it will be transmitted in the LoRa packet. If the header is implicit, it will not be transmitted
			uint8_t PayloadLength ;  //!< Size of the payload in the LoRa packet
			RadioLoRaCrcModes_t CrcMode ;  //!< Size of CRC block in LoRa packet
			RadioLoRaIQModes_t InvertIQ ; //!< Allows to swap IQ for LoRa packet
		} LoRa ;
	} Params ;                        //!< Holds the packet parameters structure
} PacketParams_t ;

/*!
 * \brief Represents the packet status for every packet type
 */
typedef struct
{
	RadioPacketTypes_t packetType ; //!< Packet to which the packet status are referring to.
	struct
	{
		struct
		{
			uint8_t RxStatus ;
			int8_t RssiAvg ;                              //!< The averaged RSSI
			int8_t RssiSync ;              //!< The RSSI measured on last packet
			uint32_t FreqError ;
		} Gfsk ;
		struct
		{
			int8_t RssiPkt ;                    //!< The RSSI of the last packet
			int8_t SnrPkt ;                      //!< The SNR of the last packet
			int8_t SignalRssiPkt ;
			uint32_t FreqError ;
		} LoRa ;
	} Params ;
} PacketStatus_t ;

/*!
 * \brief Represents the Rx internal counters values when GFSK or LoRa packet type is used
 */
typedef struct
{
	RadioPacketTypes_t packetType ; //!< Packet to which the packet status are referring to.
	uint16_t PacketReceived ;
	uint16_t CrcOk ;
	uint16_t LengthError ;
} RxCounter_t ;

/*!
 * \brief Represents a calibration configuration
 */
typedef union
{
	struct
	{
		uint8_t RC64KEnable :1 ;                      //!< Calibrate RC64K clock
		uint8_t RC13MEnable :1 ;                      //!< Calibrate RC13M clock
		uint8_t PLLEnable :1 ;                             //!< Calibrate PLL
		uint8_t ADCPulseEnable :1 ;                     //!< Calibrate ADC Pulse
		uint8_t ADCBulkNEnable :1 ;                     //!< Calibrate ADC bulkN
		uint8_t ADCBulkPEnable :1 ;                     //!< Calibrate ADC bulkP
		uint8_t ImgEnable :1 ;
		uint8_t :1 ;
	} Fields ;
	uint8_t Value ;
} CalibrationParams_t ;

/*!
 * \brief Represents a sleep mode configuration
 */
typedef union
{
	struct
	{
		uint8_t WakeUpRTC :1 ; //!< Get out of sleep mode if wakeup signal received from RTC
		uint8_t Reset :1 ;
		uint8_t WarmStart :1 ;
		uint8_t Reserved :5 ;
	} Fields ;
	uint8_t Value ;
} SleepParams_t ;

/*!
 * \brief Represents the possible radio system error states
 */
typedef union
{
	struct
	{
		uint8_t Rc64kCalib :1 ;      //!< RC 64kHz oscillator calibration failed
		uint8_t Rc13mCalib :1 ;      //!< RC 13MHz oscillator calibration failed
		uint8_t PllCalib :1 ;                    //!< PLL calibration failed
		uint8_t AdcCalib :1 ;                    //!< ADC calibration failed
		uint8_t ImgCalib :1 ;                    //!< Image calibration failed
		uint8_t XoscStart :1 ;              //!< XOSC oscillator failed to start
		uint8_t PllLock :1 ;                    //!< PLL lock failed
		uint8_t BuckStart :1 ;               //!< Buck converter failed to start
		uint8_t PaRamp :1 ;                    //!< PA ramp failed
		uint8_t :7 ;                    //!< Reserved
	} Fields ;
	uint16_t Value ;
} RadioError_t ;

typedef struct
{
	RadioIOCallbacks_t* IO_callbacks ;
	RadioCallbacks_t* callbacks ;
	/*!
	 * \brief Holds the internal operating mode of the radio
	 */
	RadioOperatingModes_t OperatingMode ;

	/*!
	 * \brief Stores the current packet type set in the radio
	 */
	RadioPacketTypes_t PacketType ;

	/*!
	 * \brief Holds a flag raised on radio interrupt
	 */
	uint8_t IrqState ;

	/*!
	 * \brief Hardware DIO IRQ functions
	 */
//	DioIrqHandler dioIrq ;
	/*!
	 * \brief Holds the polling state of the driver
	 */
	uint8_t PollingMode ;

	void (*wait_ms) ( uint32_t ms ) ;

	void (*set_interrupts) ( uint8_t enable ) ;

	void (*spi_TxRx) ( uint8_t* buf_tx, uint8_t* buf_rx, uint16_t size ) ;
	void (*spi_Tx) ( uint8_t* buf, uint16_t size ) ;
	void (*spi_Rx) ( uint8_t* buf, uint16_t size ) ;
} sx126x_t ;
/* Exported types END --------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported constants END ----------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/*!
 * \brief Sets a function to be triggered on radio interrupt
 *
 * \param [in]  irqHandler    A pointer to a function to be run on interrupt
 *                            from the radio
 */
//void IoIrqInit ( DioIrqHandler irqHandler ) ;
/*!
 * \brief DIOs interrupt callback
 *
 * \remark Called to handle all 3 DIOs pins
 */
void OnDioIrq ( sx126x_t* radio ) ;

/*!
 * \brief Initializes the radio driver
 */
void Init ( sx126x_t* radio ) ;

/*!
 * \brief Gets the current Operation Mode of the Radip
 *
 * \retval      RadioOperatingModes_t last operating mode
 */
RadioOperatingModes_t GetOperatingMode ( sx126x_t* radio ) ;

/*!
 * \brief Wakeup the radio if it is in Sleep mode and check that Busy is low
 */
void CheckDeviceReady ( sx126x_t* ) ;

/*!
 * \brief Saves the payload to be send in the radio buffer
 *
 * \param [in]  payload       A pointer to the payload
 * \param [in]  size          The size of the payload
 */
void SetPayload ( sx126x_t* radio, uint8_t *payload, uint8_t size ) ;

/*!
 * \brief Reads the payload received. If the received payload is longer
 * than maxSize, then the method returns 1 and do not set size and payload.
 *
 * \param [out] payload       A pointer to a buffer into which the payload will be copied
 * \param [out] size          A pointer to the size of the payload received
 * \param [in]  maxSize       The maximal size allowed to copy into the buffer
 */
uint8_t GetPayload ( sx126x_t* radio, uint8_t *payload, uint8_t *size,
		uint8_t maxSize ) ;

/*!
 * \brief Sends a payload
 *
 * \param [in]  payload       A pointer to the payload to send
 * \param [in]  size          The size of the payload to send
 * \param [in]  timeout       The timeout for Tx operation
 */
void SendPayload ( sx126x_t* radio, uint8_t *payload, uint8_t size,
		uint32_t timeout ) ;

/*!
 * \brief Sets the Sync Word given by index used in GFSK
 *
 * \param [in]  syncWord      SyncWord bytes ( 8 bytes )
 *
 * \retval      status        [0: OK, 1: NOK]
 */
uint8_t SetSyncWord ( sx126x_t* radio, uint8_t *syncWord ) ;

/*!
 * \brief Sets the Initial value for the LFSR used for the CRC calculation
 *
 * \param [in]  seed          Initial LFSR value ( 2 bytes )
 *
 */
void SetCrcSeed ( sx126x_t* radio, uint16_t seed ) ;

/*!
 * \brief Sets the seed used for the CRC calculation
 *
 * \param [in]  seed          The seed value
 *
 */
void SetCrcPolynomial ( sx126x_t* radio, uint16_t seed ) ;

/*!
 * \brief Sets the Initial value of the LFSR used for the whitening in GFSK protocols
 *
 * \param [in]  seed          Initial LFSR value
 */
void SetWhiteningSeed ( sx126x_t* radio, uint16_t seed ) ;

/*!
 * \brief Gets a 32 bits random value generated by the radio
 *
 * \remark The radio must be in reception mode before executing this function
 *
 * \retval randomValue    32 bits random value
 */
uint32_t GetRandom ( sx126x_t* radio ) ;

/*!
 * \brief Sets the radio in sleep mode
 *
 * \param [in]  sleepConfig   The sleep configuration describing data
 *                            retention and RTC wake-up
 */
void SetSleep ( sx126x_t* radio, SleepParams_t sleepConfig ) ;

/*!
 * \brief Sets the radio in configuration mode
 *
 * \param [in]  mode          The standby mode to put the radio into
 */
void SetStandby ( sx126x_t* radio, RadioStandbyModes_t mode ) ;

/*!
 * \brief Sets the radio in FS mode
 */
void SetFs ( sx126x_t* radio ) ;

/*!
 * \brief Sets the radio in transmission mode
 *
 * \param [in]  timeout       Structure describing the transmission timeout value
 */
void SetTx ( sx126x_t* radio, uint32_t timeout ) ;

/*!
 * \brief Sets the radio in reception Boosted mode
 *
 * \param [in]  timeout       Structure describing the transmission timeout value
 */
void SetRxBoosted ( sx126x_t* radio, uint32_t timeout ) ;

/*!
 * \brief Sets the radio in reception mode
 *
 * \param [in]  timeout       Structure describing the reception timeout value
 */
void SetRx ( sx126x_t* radio, uint32_t timeout ) ;

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void SetRxDutyCycle ( sx126x_t* radio, uint32_t rxTime, uint32_t sleepTime ) ;

/*!
 * \brief Sets the radio in CAD mode
 */
void SetCad ( sx126x_t* radio ) ;

/*!
 * \brief Sets the radio in continuous wave transmission mode
 */
void SetTxContinuousWave ( sx126x_t* radio ) ;

/*!
 * \brief Sets the radio in continuous preamble transmission mode
 */
void SetTxInfinitePreamble ( sx126x_t* radio ) ;

/*!
 * \brief Decide which interrupt will stop the internal radio rx timer.
 *
 * \param [in]  enable          [0: Timer stop after header/syncword detection
 *                               1: Timer stop after preamble detection]
 */
void SetStopRxTimerOnPreambleDetect ( sx126x_t* radio, uint8_t enable ) ;

/*!
 * \brief Set the number of symbol the radio will wait to validate a reception
 *
 * \param [in]  SymbNum          number of LoRa symbols
 */
void SetLoRaSymbNumTimeout ( sx126x_t* radio, uint8_t SymbNum ) ;

/*!
 * \brief Sets the power regulators operating mode
 *
 * \param [in]  mode          [0: LDO, 1:DC_DC]
 */
void SetRegulatorMode ( sx126x_t* radio, RadioRegulatorMode_t mode ) ;

/*!
 * \brief Calibrates the given radio block
 *
 * \param [in]  calibParam    The description of blocks to be calibrated
 */
void Calibrate ( sx126x_t* radio, CalibrationParams_t calibParam ) ;

/*!
 * \brief Calibrates the Image rejection depending of the frequency
 *
 * \param [in]  freq    The operating frequency
 */
void CalibrateImage ( sx126x_t* radio, uint32_t freq ) ;

/*!
 * \brief Sets the transmission parameters
 *
 * \param [in]  paDutyCycle     Duty Cycle for the PA
 * \param [in]  HpMax           0 for sx1261, 7 for sx1262
 * \param [in]  deviceSel       1 for sx1261, 0 for sx1262
 * \param [in]  paLUT           0 for 14dBm LUT, 1 for 22dBm LUT
 */
void SetPaConfig ( sx126x_t* radio, uint8_t paDutyCycle, uint8_t HpMax,
		uint8_t deviceSel, uint8_t paLUT ) ;

/*!
 * \brief Defines into which mode the chip goes after a TX / RX done
 *
 * \param [in]  fallbackMode    The mode in which the radio goes
 */
void SetRxTxFallbackMode ( sx126x_t* radio, uint8_t fallbackMode ) ;

/*!
 * \brief   Sets the IRQ mask and DIO masks
 *
 * \param [in]  irqMask       General IRQ mask
 * \param [in]  dio1Mask      DIO1 mask
 * \param [in]  dio2Mask      DIO2 mask
 * \param [in]  dio3Mask      DIO3 mask
 */
void SetDioIrqParams ( sx126x_t* radio, uint16_t irqMask, uint16_t dio1Mask,
		uint16_t dio2Mask, uint16_t dio3Mask ) ;

/*!
 * \brief Returns the current IRQ status
 *
 * \retval      irqStatus     IRQ status
 */
uint16_t GetIrqStatus ( sx126x_t* radio ) ;

/*!
 * \brief Indicates if DIO2 is used to control an RF Switch
 *
 * \param [in] enable     true of false
 */
void SetDio2AsRfSwitchCtrl ( sx126x_t* radio, uint8_t enable ) ;

/*!
 * \brief Indicates if the Radio main clock is supplied from a tcxo
 *
 * \param [in] tcxoVoltage     voltage used to control the TCXO
 * \param [in] timeout         time given to the TCXO to go to 32MHz
 */
void SetDio3AsTcxoCtrl ( sx126x_t* radio, RadioTcxoCtrlVoltage_t tcxoVoltage,
		uint32_t timeout ) ;

/*!
 * \brief Sets the RF frequency
 *
 * \param [in]  frequency     RF frequency [Hz]
 */
void SetRfFrequency ( sx126x_t* radio, uint32_t frequency ) ;

/*!
 * \brief Sets the radio for the given protocol
 *
 * \param [in]  packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA]
 *
 * \remark This method has to be called before SetRfFrequency,
 *         SetModulationParams and SetPacketParams
 */
void SetPacketType ( sx126x_t* radio, RadioPacketTypes_t packetType ) ;

/*!
 * \brief Gets the current radio protocol
 *
 * \retval      packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA]
 */
RadioPacketTypes_t GetPacketType ( sx126x_t* radio ) ;

/*!
 * \brief Sets the transmission parameters
 *
 * \param [in]  power         RF output power [-18..13] dBm
 * \param [in]  rampTime      Transmission ramp up time
 */
void SetTxParams ( sx126x_t* radio, int8_t power, RadioRampTimes_t rampTime ) ;

/*!
 * \brief Set the modulation parameters
 *
 * \param [in]  modParams     A structure describing the modulation parameters
 */
void SetModulationParams ( sx126x_t* radio, ModulationParams_t *modParams ) ;

/*!
 * \brief Sets the packet parameters
 *
 * \param [in]  packetParams  A structure describing the packet parameters
 */
void SetPacketParams ( sx126x_t* radio, PacketParams_t *packetParams ) ;

/*!
 * \brief Sets the Channel Activity Detection (CAD) parameters
 *
 * \param [in]  cadSymbolNum   The number of symbol to use for CAD operations
 *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
 *                              LORA_CAD_16_SYMBOL]
 * \param [in]  cadDetPeak     Limite for detection of SNR peak used in the CAD
 * \param [in]  cadDetMin      Set the minimum symbol recognition for CAD
 * \param [in]  cadExitMode    Operation to be done at the end of CAD action
 *                             [LORA_CAD_ONLY, LORA_CAD_RX, LORA_CAD_LBT]
 * \param [in]  cadTimeout     Defines the timeout value to abort the CAD activity
 */
void SetCadParams ( sx126x_t* radio, RadioLoRaCadSymbols_t cadSymbolNum,
		uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode,
		uint32_t cadTimeout ) ;

/*!
 * \brief Sets the data buffer base address for transmission and reception
 *
 * \param [in]  txBaseAddress Transmission base address
 * \param [in]  rxBaseAddress Reception base address
 */
void SetBufferBaseAddresses ( sx126x_t* radio, uint8_t txBaseAddress,
		uint8_t rxBaseAddress ) ;

/*!
 * \brief Gets the current radio status
 *
 * \retval      status        Radio status
 */
RadioStatus_t GetStatus ( sx126x_t* radio ) ;

/*!
 * \brief Returns the instantaneous RSSI value for the last packet received
 *
 * \retval      rssiInst      Instantaneous RSSI
 */
int8_t GetRssiInst ( sx126x_t* radio ) ;

/*!
 * \brief Gets the last received packet buffer status
 *
 * \param [out] payloadLength Last received packet payload length
 * \param [out] rxStartBuffer Last received packet buffer address pointer
 */
void GetRxBufferStatus ( sx126x_t* radio, uint8_t *payloadLength,
		uint8_t *rxStartBuffer ) ;

/*!
 * \brief Gets the last received packet payload length
 *
 * \param [out] pktStatus     A structure of packet status
 */
void GetPacketStatus ( sx126x_t* radio, PacketStatus_t *pktStatus ) ;

/*!
 * \brief Returns the possible system erros
 *
 * \retval sysErrors Value representing the possible sys failures
 */
RadioError_t GetDeviceErrors ( sx126x_t* radio ) ;

/*!
 * \brief Clears the IRQs
 *
 * \param [in]  irq           IRQ(s) to be cleared
 */
void ClearIrqStatus ( sx126x_t* radio, uint16_t irq ) ;

/*!
 * \brief Set the driver in polling mode.
 *
 * In polling mode the application is responsible to call ProcessIrqs( ) to
 * execute callbacks functions.
 * The default mode is Interrupt Mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * radio = SX126x( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
 * radio.Init( );
 * radio.SetPollingMode( );
 *
 * while( true )
 * {
 *                            //     IRQ processing is automatically done
 *     radio.ProcessIrqs( );  // <-- here, as well as callback functions
 *                            //     calls
 *     // Do some applicative work
 * }
 * @endcode
 *
 * \see SX126x::SetInterruptMode
 */
void SetPollingMode ( sx126x_t* radio ) ;

/*!
 * \brief Set the driver in interrupt mode.
 *
 * In interrupt mode, the driver communicate with the radio during the
 * interruption by direct calls to ProcessIrqs( ). The main advantage is
 * the possibility to have low power application architecture.
 * This is the default mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * radio = SX126x( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
 * radio.Init( );
 * radio.SetInterruptMode( );   // Optionnal. Driver default behavior
 *
 * while( true )
 * {
 *     // Do some applicative work
 * }
 * @endcode
 *
 * \see SX126x::SetPollingMode
 */
void SetInterruptMode ( sx126x_t* radio ) ;

/*!
 * \brief Resets the radio
 */
void Reset ( sx126x_t* radio ) ;

/*!
 * \brief Wake-ups the radio from Sleep mode
 */
void Wakeup ( sx126x_t* radio ) ;

/*!
 * \brief Writes the given command to the radio
 *
 * \param [in]  opcode        Command opcode
 * \param [in]  buffer        Command parameters byte array
 * \param [in]  size          Command parameters byte array size
 */
void WriteCommand ( sx126x_t* radio, RadioCommands_t opcode, uint8_t *buffer,
		uint16_t size ) ;

/*!
 * \brief Reads the given command from the radio
 *
 * \param [in]  opcode        Command opcode
 * \param [in]  buffer        Command parameters byte array
 * \param [in]  size          Command parameters byte array size
 */
void ReadCommand ( sx126x_t* radio, RadioCommands_t opcode, uint8_t *buffer,
		uint16_t size ) ;

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [in]  address       First Radio register address
 * \param [in]  buffer        Buffer containing the new register's values
 * \param [in]  size          Number of registers to be written
 */
void WriteRegister ( sx126x_t* radio, uint16_t address, uint8_t *buffer,
		uint16_t size ) ;

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [in]  address       Register address
 * \param [in]  value         New register value
 */
void WriteReg ( sx126x_t* radio, uint16_t address, uint8_t value ) ;

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [in]  address       First Radio register address
 * \param [out] buffer        Buffer where to copy the registers data
 * \param [in]  size          Number of registers to be read
 */
void ReadRegister ( sx126x_t* radio, uint16_t address, uint8_t *buffer,
		uint16_t size ) ;

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [in]  address       Register address
 *
 * \retval      data          Register value
 */
uint8_t ReadReg ( sx126x_t* radio, uint16_t address ) ;

/*!
 * \brief Writes Radio Data Buffer with buffer of size starting at offset.
 *
 * \param [in]  offset        Offset where to start writing
 * \param [in]  buffer        Buffer pointer
 * \param [in]  size          Buffer size
 */
void WriteBuffer ( sx126x_t* radio, uint8_t offset, uint8_t *buffer,
		uint8_t size ) ;

/*!
 * \brief Reads Radio Data Buffer at offset to buffer of size
 *
 * \param [in]  offset        Offset where to start reading
 * \param [out] buffer        Buffer pointer
 * \param [in]  size          Buffer size
 */
void ReadBuffer ( sx126x_t* radio, uint8_t offset, uint8_t *buffer,
		uint8_t size ) ;

/*!
 * \brief Gets the current status of the radio DIOs
 *
 * \retval      status        [Bit #3: DIO3, Bit #2: DIO2,
 *                             Bit #1: DIO1, Bit #0: BUSY]
 */
//uint8_t GetDioStatus ( sx126x_t* radio ) ;

/*!
 * \brief Returns the device type
 *
 * \retval      0: SX1261, 1: SX1262, 2: SX1268
 */
uint8_t GetDeviceType ( sx126x_t* radio ) ;

/*!
 * \brief Returns the matching frequency
 *
 * \retval      1: 868 MHz
 *              0: 915 MHz
 */
uint8_t GetFreqSelect ( sx126x_t* radio ) ;

/*!
 * \brief RF Switch power on
 */
void AntSwOn ( sx126x_t* radio ) ;

/*!
 * \brief RF Switch power off
 */
void AntSwOff ( sx126x_t* radio ) ;

/*!
 * \brief Process the analysis of radio IRQs and calls callback functions
 *        depending on radio state
 */
void ProcessIrqs ( sx126x_t* radio ) ;
/* Exported functions prototypes END -----------------------------------------*/

#ifdef __cplusplus
}
#endif	// extern "C"

#endif /* SX126X_H_ */